/*
  Controller.cpp - Library for a PID controller.
  Group 2: Freeman Porten, Lachlan Barnes, Jake Olliff, Calvin Lee

  Controller: a class that controls is responsible for the overal
  control systems of the robot. It's contain methods such as InitForWall
  and WallFollow which are the initilizaion and wall following programs
  that take sensor readings and return desired PID actions.
*/

#include "Controller.h"

/**
 * Controller Initializer
 * 
 * Arduinio doesn't create one by default so one needs to be provided
 * even if it isn't used
 */
Controller::Controller()
{
}

/**
 * Initilize the robots position
 * 
 * This function reads the side-mounted IR sensor and turns the robot
 * CCW until the difference between the IR readings is below a value in
 * which it's return value changes to 1. This function assumes the sensors
 * are side mounted on the right aspect of the robot otherwise out[2] 
 * will need to be changed.
 */
bool Controller::InitForWall(double frontIR, double backIR, double out[3])
{
  double IRTolerence = 5;
  bool finished = abs(frontIR - backIR) < IRTolerence;
  out[0] = 0;
  out[1] = 0;
  out[2] = 20; // Turning speed initially, CCW
  return finished ? 1 : 0;
}

/**
 * Drive the robot forward parrallel to a wall
 * 
 * This function reads the side-mounted IR sensor and calculates
 * the appropriate (y and omega) PID output values needed to A) 
 * drive a set (15cm) distance from a surface and B) correct
 *  any deviations from being parrallel from that surface.
 */
void Controller::WallFollow(double frontIR, double backIR, double targetDistance, double out[3])
{
  float l_IR = 185; //Distance between IR sensors
  out[1] = targetDistance - (frontIR + backIR) / 2;
  out[2] = (frontIR - backIR) / (2 * l_IR);
  return;
}

/**
 * Detect obstacles ahead and adjust x PID effort accordingly 
 * 
 * This function reads the side-mounted IR sensor and calculates
 * the appropriate PID output values needed to A) drive a set 
 * (15cm) distance from a surface but also correct any deviations 
 * from being parrallel from the surface.
 */
void Controller::FrontDetect(double sonar, double targetDistance, double out[3])
{
  out[0] = sonar - targetDistance;

  return;
}
