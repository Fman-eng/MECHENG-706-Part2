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

#define WALL_FOLLOW_DISTANCE 145
#define WALL_STOP_DISTANCE 40

enum State{
  INITALIZE,
  WALLFOLLOW,
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
  IRSensor OIRFront(A5, 3);
  IRSensor OIRBack(A7, 1);
  SonarSensor sonar(48, 49);

  // Init averaged sensor values
  float frontAvg = 0, rearAvg = 0, obsFrontAvg = 0, obsRearAvg = 0;

  // Intstantiated the Controller
  Controller mainController;

  // Instantiate and initialise Drive
  Drive drive(46, 47, 50, 51);
  drive.Init();

  // Initalisation variables
  bool init_finished = false;

  // Corner counter
  int cornerCount = 0;

  // Flag for Intialising and finding the wall
  int isIntialising = 1;

  // Flag for open-loop cornering
  int isTurning = 0;

  // Flag for wall alignment after corner
  int isAligning = 0;

  // Timer to wait till robot is aligned
  float alignTimer;

  // Flag for finished course
  int isDone = 0;
  
  // state machine
  State state = INITALIZE;

  float sonarDist;

//   int sensorDiff=0, sensorAvg=0, detectionThreshold=100, distToWall=1500, stopDist=50;
// //  while((sensorAvg < stopDist) & (sensorDiff < detectionThreshold)){
//   while(1){
//     //############## DELETE WHEN FINISHED ###############
//     state = FIREAPPROCH;
//     PIDVx.SetMode(AUTOMATIC);
//     PIDVy.SetMode(AUTOMATIC);
//     PIDW.SetMode(AUTOMATIC);
//     // Detect an obstical
//     //Serial.println(abs(IRFront.getDistance()-IRBack.getDistance()));
//     Serial.print(IRFrontOS.getDistance());
//     Serial.print(",");
//     Serial.println(IRBack.getDistance());
//     sensorDiff = abs(IRFrontOS.getDistance()-IRBack.getDistance());
//     sensorAvg = (IRFrontOS.getDistance()-IRBack.getDistance())/2;
//     pidIn[0] = 0;
//     pidIn[1] = -5;
//     pidIn[2] = 0;
//     if((sensorDiff < detectionThreshold) & sensorAvg < distToWall){
//       //drive forward
//       Serial.println("Obstical Detected!");
//     }
//     else if((IRFrontOS.getDistance()-IRBack.getDistance()) > detectionThreshold){
//       Serial.println("Drive 1"); 
//       pidIn[0] = 2;
//     } else if((IRBack.getDistance()-IRFrontOS.getDistance()) > detectionThreshold){
//       // drive xx
//       Serial.println("Drive 2");
//       pidIn[0] = -2;
//     } else if (sensorDiff < detectionThreshold & sensorAvg > distToWall){
//       Serial.println("Lost obstical");
//       pidIn[0] = 0;
//     }
//     PIDVx.Compute();
//     PIDVy.Compute();
//     PIDW.Compute();
//     drive.SetSpeedThroughKinematic(pidOut[0], pidOut[1], pidOut[2]);
//     delay(50);
//   }

  // Super Loop
  while (1)
  {
    // ################ DELETE WHEN FINISHED ##################
    state = FIREAPPROCH;
    delay(100);
    // ########################################################
    frontAvg = IRFront.getAverage();
    rearAvg = IRBack.getAverage();
    obsFrontAvg = OIRFront.getAverage(); 
    obsRearAvg = OIRBack.getAverage();

    switch(state){
      case INITALIZE:
      {
        //Code for intialisation sequence
        //Intilisation code
        Serial.println("Intialising");
        if (mainController.InitForWall(frontAvg, rearAvg, pidOut))
        {
          isAligning = 1;
          isIntialising = 0;
          alignTimer = millis();
        }

        PIDVx.SetMode(MANUAL);
        PIDVy.SetMode(MANUAL);
        PIDW.SetMode(MANUAL);
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
        sonarDist = sonar.getDistance();
        mainController.WallFollow(frontAvg, rearAvg, WALL_FOLLOW_DISTANCE, pidIn);
        mainController.FrontDetect(sonarDist, WALL_STOP_DISTANCE, pidIn);

        /* Check if the PIDs need to be computed, the PIDs run at 50Hz which is
          slower than the super loop. Every few loops the PIDs will be recalulated,
          this ensures that the timestep stay constant prefencting issues with the
          intergrator and derivitive term */
        PIDVx.SetMode(AUTOMATIC);
        PIDVy.SetMode(AUTOMATIC);
        PIDW.SetMode(AUTOMATIC);

        /* Check if the next wall has been reached, increment the corner
          counter and turn the next corner.*/
        if (sonarDist <= WALL_STOP_DISTANCE + 5)
        {
          Serial.println("WALL DETECTED!");
          state = WALLTURN;
        }
        break;
      }
      case FIRECHECK:
      {
        Serial.println("firecheck");
        break;
      }
      case FIREAPPROCH:
      {
        // Serial.println("fireapproach");
        PIDVx.SetMode(AUTOMATIC);
        PIDVy.SetMode(AUTOMATIC);
        PIDW.SetMode(AUTOMATIC);

        int distToWall =  100;
        int obsticalDiff = 50;
        int diffIR = abs(obsFrontAvg - obsRearAvg);
        int avgIR = abs(obsFrontAvg - obsRearAvg)/2;

        pidIn[0] = 0;
        if(avgIR > 5){
          pidIn[1] = -5;
        }
        pidIn[2] = 0;
        
        // Serial.println("IR VALUES");
        Serial.print(obsFrontAvg);
        Serial.print(",");
        Serial.print(obsRearAvg);
        Serial.print(",");
        Serial.print(diffIR);
        Serial.print(",");
        Serial.println(avgIR);

        if((diffIR < obsticalDiff) & (avgIR < distToWall)){
        // Both IR's detect a obstical
          Serial.println("Both sensors detected an obstical");
        } else if((diffIR < obsticalDiff) & (avgIR > distToWall)){
        // None detect
          Serial.println("Neither sensors detected an obstical");
        } else if(diffIR > obsticalDiff){
          // An obstical detected
          if(obsFrontAvg < obsRearAvg){
          // The LHS IR sensor has detected an objected
            Serial.println("LHS sensor detected an obstical");
            pidIn[0] = 2;
          } else if(obsRearAvg < obsFrontAvg){
          // The RHS IR sensor has detected an objected
            Serial.println("RHS sensor detected an obstical");
            pidIn[0] = -2;
          }
        }
        break;
      }
      case FIREEXTINGUISH:
      {
              Serial.println("Fireextinguish");
        break;
      }
      case WALLRETURN:
      {
        Serial.println("wallreturn");

        break;
      }
      case WALLTURN:
      {
        // Code for turning at the corners
        Serial.println("is turning");
        // PIDVx.SetMode(MANUAL);
        // PIDVy.SetMode(MANUAL);
        // PIDW.SetMode(MANUAL);
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
