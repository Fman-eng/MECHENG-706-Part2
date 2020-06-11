/*
  main.ino - Main file for arduino based wall following program.
  Group 2: Freeman Porten, Lachlan Barnes, Jake Olliff, Calvin Lee
*/
#include "Controller.h"
#include "Drive.h"
#include "IRSensor.h"
#include "SonarSensor.h"
#include "PID_v1.h"
#include "Phototransistors.h"
#include "NewPing.h"

#define WALL_FOLLOW_DISTANCE 145
#define WALL_STOP_DISTANCE 50
#define OBS_DETECT_DISTANCE 300
#define FIRE_THRESHHOLD 250
enum State{
  INITALIZE,
  WALLFOLLOW,
  WALLIGNORE,
  FIRECHECK,
  FIREAPPROCH,
  FIREEXTINGUISH,
  WALLRETURN,
  WALLTURN,
  COMPLETE
};



// pidIn is the position array input to the PID controllers
double pidIn[3];
// pidOut is the output velocity vector array of the PID controllers
double pidOut[3];
/* setPoint contains the values the Controllers will attempt to drive
the error to, in our case this will be zero. */
double setPoints[3];

// Setup PID controller instances
PID PIDVx(&pidIn[0], &pidOut[0], &setPoints[0], 200, 0, 0, REVERSE);
PID PIDVy(&pidIn[1], &pidOut[1], &setPoints[1], 300, 0, 0, REVERSE);
PID PIDW(&pidIn[2], &pidOut[2], &setPoints[2], 300, 0, 0, REVERSE);

void setup()
{
  Serial.begin(9600);
  Phototransistors pt(A9, A10, A8, A8);
  pinMode(12, OUTPUT);

  /* These prevent Intergrator windup by stoping the intergrator summing
  if the output goes outside of the range specified below. */
  PIDVx.SetOutputLimits(-8000, 8000);
  PIDVy.SetOutputLimits(-8000, 8000);
  PIDW.SetOutputLimits(-2000, 2000);

  /* Begin the PIDs in manual mode as we start with Open-loop control
  to find the starting wall. */
  PIDVx.SetMode(MANUAL);
  PIDVy.SetMode(MANUAL);
  PIDW.SetMode(MANUAL);

  /* Intialise the PID inputs and outputs to zero. The setPoint of each
  PID controller are zero as we want to drive error to zero*/
  for (int i = 0; i < 3; ++i)
  {
    pidIn[i] = 0;
    pidOut[i] = 0;
    setPoints[i] = 0;
  }

  // Sensor Instantiation
  IRSensor IRFront(A15, 0);
  IRSensor IRBack(A14, 2);
  NewPing sonar(48, 49, 1000);
  IRSensor OIRBack(A7, 1);
  IRSensor OIRFront(A5, 3);

  // Init averaged sensor values
  float frontAvg = 0, rearAvg = 0, obsFrontAvg = 0, obsRearAvg = 0, sonarDist = 0;

  // Intstantiated the Controller
  Controller mainController;

  // Instantiate and initialise Drive
  Drive drive(46, 47, 50, 51);
  drive.Init();
  
  // state machine
  State state = WALLFOLLOW;
  // while(1){
  //   Serial.println(pt.FireDetected(FIRE_THRESHHOLD));
  //   delay(1);
  // }
  // Super Loop
  while (1)
  {
    // ################ DELETE WHEN FINISHED ##################
    //state = FIRECHECK;
    //delay(100);
    // ########################################################
    frontAvg = IRFront.getAverage();
    rearAvg = IRBack.getAverage();
    obsFrontAvg = OIRFront.getAverage()*10; 
    obsRearAvg = OIRBack.getAverage()*10;


    switch(state){
      case INITALIZE:
      {
        //Code for intialisation sequence
        //Intilisation code
        Serial.println("Intialising");

        PIDVx.SetMode(MANUAL);
        PIDVy.SetMode(MANUAL);
        PIDW.SetMode(MANUAL);
        if (mainController.InitForWall(frontAvg, rearAvg, pidOut))
        {
        }
        state = WALLFOLLOW;
        break;
      }
      case WALLFOLLOW:
      {
        Serial.println("wall following");
        /* This sets the value of Vy and Wz in the velocities array by using the
          IR sensors to meaure its distance and angle from the wall. The wall follow
          is set to 145mm to account for the location of the IR sensorson the robot.
          front detect is set to have the robot stop 40mm from the next wall*/
        sonarDist = sonar.ping_cm()*10;
        //sonarDist = sonar.getDistance();
        Serial.println(sonarDist);
        mainController.WallFollow(frontAvg, rearAvg, WALL_FOLLOW_DISTANCE, pidIn);
        mainController.FrontDetect(sonarDist, WALL_STOP_DISTANCE, pidIn);

        /* Check if the PIDs need to be computed, the PIDs run at 50Hz which is
          slower than the super loop. Every few loops the PIDs will be recalulated,
          this ensures that the timestep stay constant prefencting issues with the
          intergrator and derivitive term */
        PIDVx.SetMode(AUTOMATIC);
        PIDVy.SetMode(AUTOMATIC);
        PIDW.SetMode(AUTOMATIC);

        /* Check if the next wall has been reached, Change the state to wallturn*/
        if (sonarDist <= WALL_STOP_DISTANCE)
        {
          Serial.println("WALL DETECTED!");
          state = WALLTURN;
        }

        if (obsRearAvg <= OBS_DETECT_DISTANCE)
        {
          Serial.println("OBSTACLE DETECTED!");
          Serial.println(obsRearAvg);
          state = FIRECHECK;
        }

        break;
      }
      // case WALLIGNORE:
      // {
      //   Serial.println("obstacle ignoring");
      //   /* This sets the value of Vy and Wz in the velocities array by using the
      //     IR sensors to meaure its distance and angle from the wall. The wall follow
      //     is set to 145mm to account for the location of the IR sensorson the robot.
      //     front detect is set to have the robot stop 40mm from the next wall*/
      //   sonarDist = sonar.ping_cm()*10;
      //   //sonarDist = sonar.getDistance();
      //   Serial.println(sonarDist);
      //   mainController.WallFollow(frontAvg, rearAvg, WALL_FOLLOW_DISTANCE, pidIn);
      //   mainController.FrontDetect(sonarDist, WALL_STOP_DISTANCE, pidIn);

      //   /* Check if the PIDs need to be computed, the PIDs run at 50Hz which is
      //     slower than the super loop. Every few loops the PIDs will be recalulated,
      //     this ensures that the timestep stay constant prefencting issues with the
      //     intergrator and derivitive term */
      //   PIDVx.SetMode(AUTOMATIC);
      //   PIDVy.SetMode(AUTOMATIC);
      //   PIDW.SetMode(AUTOMATIC);

      //   /* Check if the next wall has been reached, Change the state to wallturn*/
      //   if (sonarDist <= WALL_STOP_DISTANCE)
      //   {
      //     Serial.println("WALL DETECTED!");
      //     state = WALLTURN;
      //   }

      //   if (obsRearAvg <= OBS_DETECT_DISTANCE)
      //   {
      //     Serial.println("OBSTACLE DETECTED!");
      //     Serial.println(obsRearAvg);
      //     state = WALLFOLLOW;
      //   }

      //   break;
      }
      case FIRECHECK:
      {
        // Serial.println("firecheck");
        // /* This sets the value of Vy and Wz in the velocities array by using the
        //   IR sensors to meaure its distance and angle from the wall. The wall follow
        //   is set to 145mm to account for the location of the IR sensorson the robot.
        //   front detect is set to have the robot stop 40mm from the next wall*/
        // sonarDist = sonar.ping_cm()*10;
        // //sonarDist = sonar.getDistance();
        // Serial.println(sonarDist);
        // mainController.WallFollow(frontAvg, rearAvg, WALL_FOLLOW_DISTANCE, pidIn);
        // mainController.FrontDetect(sonarDist, WALL_STOP_DISTANCE, pidIn);

        // /* Check if the PIDs need to be computed, the PIDs run at 50Hz which is
        //   slower than the super loop. Every few loops the PIDs will be recalulated,
        //   this ensures that the timestep stay constant prefencting issues with the
        //   intergrator and derivitive term */
        // PIDVx.SetMode(AUTOMATIC);
        // PIDVy.SetMode(AUTOMATIC);
        // PIDW.SetMode(AUTOMATIC);

        // /* Check if the next wall has been reached, Change the state to wallturn*/
        // if (sonarDist <= WALL_STOP_DISTANCE)
        // {
        //   Serial.println("WALL DETECTED!");
        //   state = WALLTURN;
        // }

        // if ((obsRearAvg >= OBS_DETECT_DISTANCE) && (obsRearAvg >= OBS_DETECT_DISTANCE) )
        // {
        //   Serial.println("OBSTACLE Passed!");
        //   Serial.println(obsRearAvg);
        //   state = WALLFOLLOW;
        // }
        
        Serial.println(pt.FireDetected(FIRE_THRESHHOLD)); 
        if(pt.FireDetected(FIRE_THRESHHOLD)){
          state = FIREAPPROCH;
        }else
        {
          state = WALLFOLLOW;
        }
        
        break;
      }
      case FIREAPPROCH:
      {
        // Serial.println("fireapproach");
        PIDVx.SetMode(MANUAL);
        PIDVy.SetMode(MANUAL);
        PIDW.SetMode(MANUAL);

        int distToWall =  50;
        int tolerance = 20;
        int stopDist = 20;
        int diffIR = abs(obsFrontAvg - obsRearAvg);
        int avgIR = (obsFrontAvg + obsRearAvg)/2;

        pidOut[0] = 0;
        if((obsFrontAvg < stopDist) || (obsRearAvg < stopDist)){
        //if(avgIR <= stopDist){
          pidOut[0]=0;
          pidOut[1]=0;
          pidOut[2]=0;
          state=FIREEXTINGUISH;
        } else {
          pidOut[1] = 8000;
        }
        pidOut[2] = 0;
        
        

        if((diffIR < tolerance) & (avgIR < distToWall)){
        // Both IR's detect a obstical
          // Serial.println("Both sensors detected an obstical");
        } else if((diffIR < tolerance) & (avgIR > distToWall)){
        // None detect
          // Serial.println("Neither sensors detected an obstical");
        } else if(diffIR > tolerance){
          // An obstical detected
          if(obsFrontAvg < obsRearAvg){
          // The LHS IR sensor has detected an objected
            // Serial.println("LHS sensor detected an obstical");
            pidOut[0] = 6000;
          } else if(obsRearAvg < obsFrontAvg){
          // The RHS IR sensor has detected an objected
            // Serial.println("RHS sensor detected an obstical");
            pidOut[0] = -6000;
          }
        }
        break;
      }
      case FIREEXTINGUISH:
      {
        // Set motor values to zero
        pidOut[0] = 0;
        pidOut[1] = 0;
        pidOut[2] = 0;
        drive.SetSpeedThroughKinematic(pidOut[0], pidOut[1], pidOut[2]);

        Serial.println("Fireextinguish");
        digitalWrite(12, HIGH);
        delay(10000);
        digitalWrite(12, LOW);
        state=WALLRETURN;
        break;

        // if(pt.FireDetected(FIRE_THRESHHOLD)){
        //   int timer;
        //   digitalWrite(12,HIGH);
        // }
        // else{
        //   state = WALLFOLLOW;
        // }
        // break;
      }
      case WALLRETURN:
      {
        Serial.println("wallreturn");
        PIDVx.SetMode(MANUAL);
        PIDVy.SetMode(MANUAL);
        PIDW.SetMode(MANUAL);
        if(((frontAvg+rearAvg)/2) > WALL_FOLLOW_DISTANCE){
          pidOut[1] = -5000;
        } else{
          pidOut[1] = 0;
          state = WALLFOLLOW;
        }
        break;
      }
      case WALLTURN:
      {
        // Code for turning at the corners
        Serial.println("is turning");
        PIDVx.SetMode(MANUAL);
        PIDVy.SetMode(MANUAL);
        PIDW.SetMode(MANUAL);
        drive.EnableMotors();
        drive.RotateOL(500, 90);
        state = WALLFOLLOW;
        break;
      }
      case COMPLETE:
      {
        /* When the end point is reached stop the motors*/
        Serial.println("Program Finished");
        // Program finished
        PIDVx.SetMode(MANUAL);
        PIDVy.SetMode(MANUAL);
        PIDW.SetMode(MANUAL);
        drive.DisableMotors();
        break;
      }
      default:
       {
        Serial.println("defaulting");

      break;
      }
    }

    PIDVx.Compute();
    PIDVy.Compute();
    PIDW.Compute();

    /* This applies the inverse Kinimatic equations to the motors using
    the Vx, Vy, Wz stored in the velocity vector that the PID outputs.*/
    drive.SetSpeedThroughKinematic(pidOut[0], pidOut[1], pidOut[2]);
  }
}

// Unused in our implimentation
void loop()
{
  // put your main code here, to run repeatedly:
}
