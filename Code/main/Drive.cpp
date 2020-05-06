/*
  Drive.cpp - Source file for the Drive class for an arduino based robot.
  Group 2: Freeman Porten, Lachlan Barnes, Jake Olliff, Calvin Lee
*/
#include "Drive.h"

/**
 * Contructor for the Drive class
 * 
 * Takes the pin allocation of the robots 4 servo motors as inputs and
 * makes the 4 class variables corrisponding to the 4 servo motors equal
 * to the input pin allocations.
 */
Drive::Drive(byte leftFront, byte leftRear, byte rightFront, byte rightRear)
{
  Serial.println("Init drive class!");
  this->leftFront = leftFront;
  this->leftRear = leftRear;
  this->rightFront = rightFront;
  this->rightRear = rightRear;
}

/**
 * Initializes and enables the robots 4 servo motors
 * 
 * Sets the 4 servo motor pins as outputs and calls
 * the EnableMotors function which attaches 4 Servo
 * objects to the 4 pins set as outputs.
 */
void Drive::Init()
{
  pinMode(leftFront, OUTPUT);
  pinMode(leftRear, OUTPUT);
  pinMode(rightFront, OUTPUT);
  pinMode(rightRear, OUTPUT);
  EnableMotors();
}

/**
 * Enable/attach the 4 servo motors
 * 
 * Attach the 4 servo class variables to the 4 pins
 * which are wiried to the servo motors. 
 */
void Drive::EnableMotors()
{
  this->leftFrontMotor.attach(leftFront);
  this->leftRearMotor.attach(leftRear);
  this->rightFrontMotor.attach(rightFront);
  this->rightRearMotor.attach(rightRear);
}

/**
 * Disable the 4 servo motors
 * 
 * Disattach the 4 servo class variables to the 4 pins
 * which are wiried to the servo motors. 
 */
void Drive::DisableMotors()
{
  this->leftFrontMotor.detach();
  this->leftRearMotor.detach();
  this->rightFrontMotor.detach();
  this->rightRearMotor.detach();
}

/**
 * Rotate the robot with closed loop control
 * 
 * Given a specified turn speed and angle, rotate
 * the robote using a closed loop controller with a
 * gyroscope as the sensor.
 */
void Drive::RotatePID(int turnSpeed, int angle)
{
  //Use gyro and PID controller to control rotation (not needed in A1 implementation)
}

/**
 * Rotate the robot using open looped control
 * 
 * Take an turn speed and angle and turn the robot for
 * that angle. Positive angle (i.e. 90) corrisponds
 * to clockwise rotation of the robot.
 */
void Drive::RotateOL(int turnSpeed, int angle)
{
  long startTime = millis();
  float angleToTime = 10;
  while (millis() < startTime + angleToTime * abs(angle))
  {
    int turnDirection = angle / abs(angle);
    this->leftFrontMotor.writeMicroseconds(1500 + turnSpeed * turnDirection);
    this->leftRearMotor.writeMicroseconds(1500 + turnSpeed * turnDirection);
    this->rightFrontMotor.writeMicroseconds(1500 + turnSpeed * turnDirection);
    this->rightRearMotor.writeMicroseconds(1500 + turnSpeed * turnDirection);
  }
}

/**
 * Control the robots movements using it's kinematic equations
 * 
 * Specify the desired velocity in the x and y (vx and vy)
 * and the desired rotation (omega) and based on the robots
 * kinematic equations the appropriate speed will be sent to 
 * each of it's servo motors. A scalar is applied to each
 * motor to keep each motor out of its saturation region,
 * this also maintains the relative speed between each motor
 * ensuring that no single input signal dominates the others
 * in control actuation.
 */
void Drive::SetSpeedThroughKinematic(float vx, float vy, float omega)
{
  float maxSpeed = 750;
  float wheelRadius = 28; //  wheel radius in mm
  float lx = 80;
  float ly = 90;
  float saturateScaler = 1; //  Scaler to maintain relative speeds between motors while saturated

  if ((abs(vx) + abs(vy) + abs((lx + ly) * omega)) / wheelRadius >= maxSpeed)
    saturateScaler = maxSpeed * wheelRadius / (abs(vx) + abs(vy) + abs((lx + ly) * omega));
  this->leftFrontMotor.writeMicroseconds(1500 + saturateScaler * (vx + vy - (lx + ly) * omega) / wheelRadius);
  this->leftRearMotor.writeMicroseconds(1500 + saturateScaler * (vx - vy - (lx + ly) * omega) / wheelRadius);
  this->rightRearMotor.writeMicroseconds(1500 - saturateScaler * (vx - vy + (lx + ly) * omega) / wheelRadius);
  this->rightFrontMotor.writeMicroseconds(1500 - saturateScaler * (vx + vy + (lx + ly) * omega) / wheelRadius);
}

/**
 * Drive the robot forward at a specified speed
 */
void Drive::Forward()
{
  Serial.println("Driving forward");
  this->leftFrontMotor.writeMicroseconds(1500 + this->speedVal);
  this->leftRearMotor.writeMicroseconds(1500 + this->speedVal);
  this->rightFrontMotor.writeMicroseconds(1500 - this->speedVal);
  this->rightRearMotor.writeMicroseconds(1500 - this->speedVal);
}

/**
 * Stop the robot from moving in any direction
 */
void Drive::Halt()
{
  Serial.println("Halting");
  this->leftFrontMotor.writeMicroseconds(1500);
  this->leftRearMotor.writeMicroseconds(1500);
  this->rightFrontMotor.writeMicroseconds(1500);
  this->rightRearMotor.writeMicroseconds(1500);
  Serial.println("Ending Halt");
}
