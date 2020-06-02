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
  Initalize,
  WallFollow,
  FireCheck,
  FireApproch,
  FireExtinguish,
  WallReturn,
  Turning
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
PID PIDVy(&pidIn[1], &pidOut[1], &setPoints[1], 170, 483, 40, REVERSE);
PID PIDW(&pidIn[2], &pidOut[2], &setPoints[2], 130, 420, 27, REVERSE);

void setup()
{
  Serial.begin(9600);
  Phototransistors pt(A9, A10, A8, A8);
  pinMode(12, OUTPUT);
  while(1){
    if(pt.FireDetected()){
      digitalWrite(12, HIGH);
     } else{
      digitalWrite(12, LOW);
     }
  }

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
  IRSensor IRFront(A14, true);
  IRSensor IRBack(A15, false);
  SonarSensor sonar(48, 49);

  // Intstantiated the Controller
  Controller mainController;

  // Instantiate and initialise Drive
  Drive drive(46, 47, 50, 51);
  drive.Init();

  // Initalisation variables
  bool init_finished = false;

  // Setup for the 4th order FIR filter
  float frontIRValues[5];
  float rearIRValues[5];
  for (int i = 0; i < 5; i++)
  {
    frontIRValues[i] = IRFront.getDistance();
    rearIRValues[i] = IRBack.getDistance();
  }
  float frontAvg;
  float rearAvg;
  int firItr = 0;

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
  
  // Super Loop
  while (1)
  {

    /* Use a shift register to store the previous values of the IR sensors
    to apply a fourth order FIR filter, this prevents noise interfering
    with the derivative terms of the PID controllers. firItr iterates
    through each value in the arrays and updates them with the new
    values from the sensors. The arrays are then averaged bfore being
    input into the controller*/
    frontIRValues[firItr] = IRFront.getDistance();
    rearIRValues[firItr] = IRBack.getDistance();
    firItr = (firItr + 1) % 5;
    frontAvg = 0;
    rearAvg = 0;
    for (int i = 0; i < 5; i++)
    {
      frontAvg += frontIRValues[i];
      rearAvg += rearIRValues[i];
    }
    frontAvg = frontAvg / 5;
    rearAvg = rearAvg / 5;

    //Intilisation code
    if (isIntialising)
    {
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
    }
    /* This contains the logic for the open-loop turning at the corners*/
    else if (isTurning)
    {
      Serial.println("is turning");
      PIDVx.SetMode(MANUAL);
      PIDVy.SetMode(MANUAL);
      PIDW.SetMode(MANUAL);
      drive.RotateOL(500, 90);
      isAligning = 1;
      alignTimer = millis();
    }
    // Align robot without moving forward before leaving corner
    else if (isAligning)
    {
      Serial.println("aligning");
      mainController.WallFollow(frontAvg, rearAvg, WALL_FOLLOW_DISTANCE, pidIn);
      pidIn[0] = 0;
      pidOut[0] = 0;

      /* Check if the PIDs need to be computed, the PIDs run at 50Hz which is
      slower than the super loop. Every few loops the PIDs will be recalulated,
      this ensures that the timestep stay constant prefencting issues with the
      intergrator and derivitive term */
      PIDVx.SetMode(MANUAL);
      PIDVy.SetMode(AUTOMATIC);
      PIDW.SetMode(AUTOMATIC);

      // Wait till PID has finished aligning
      if ((abs(frontAvg - rearAvg) > 5) || (abs(WALL_FOLLOW_DISTANCE - (frontAvg + rearAvg) / 2) > 5))
      {
        alignTimer = millis();
      }
      // Waits till the robot is alligned for 0.25 sec
      if (millis() - alignTimer >= 250)
        isAligning = 0;
    }

    // Go round 4 corners
    else if (cornerCount <= 4)
    {
      Serial.println("wall following");
      /* This sets the value of Vy and Wz in the velocities array by using the
        IR sensors to meaure its distance and angle from the wall. The wall follow
        is set to 145mm to account for the location of the IR sensorson the robot.
        front detect is set to have the robot stop 40mm from the next wall*/
      float sonarDist = sonar.getDistance();
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
        if (cornerCount < 4)
          isTurning = 2;
        cornerCount++;
      }
    }
    /* When the end point is reached stop the motors*/
    else
    {
      Serial.println("Program Finished");
      // Program finished
      PIDVx.SetMode(MANUAL);
      PIDVy.SetMode(MANUAL);
      PIDW.SetMode(MANUAL);
      drive.Halt();
      drive.DisableMotors();
    }

    PIDVx.Compute();
    PIDVy.Compute();
    PIDW.Compute();

    /*  Check for OL control, do not use the
    closed loop control if isTurning is true*/
    if (isTurning)
    {
      isTurning--;
    }
    else
    {
      /* This applies the inverse Kinimatic equations to the motors using
        the Vx, Vy, Wz stored in the velocity vector that the PID outputs.*/
      drive.SetSpeedThroughKinematic(pidOut[0], pidOut[1], pidOut[2]);
    }
  }
}

// Unused in our implimentation
void loop()
{
  // put your main code here, to run repeatedly:
}
