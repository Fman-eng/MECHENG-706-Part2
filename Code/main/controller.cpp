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
  out[2] = (frontIR - backIR)/(2 * l_IR);
  //Serial.println(out[2]);
  return;
}

/**
 * Drive the robot forward parrallel to a wall
 * 
 * This function reads the side-mounted IR sensor and calculates
 * the appropriate (y and omega) PID output values needed to A) 
 * drive a set (15cm) distance from a surface and B) correct
 *  any deviations from being parrallel from that surface.
 */
void Controller::GyroTurn(double gyroAngle, double targetAngle, double out[3])
{
  float l_IR = 185; //Distance between IR sensors
  out[2] = targetAngle - gyroAngle;
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

void Controller::GapScan(double Range[359], int angle, double distance)
{
  Range[angle] = distance;
}

void Controller::GapFill(double Range[359])
{
  for (int i = 0; i < 359; ++i)
  {
    if (Range[i] == 0)
    {
      if (i == 0)
      {
        if ((Range[359]-Range[1]) > 20)
        {
          Range[i] = Range[1];
        }
        else
        {
          Range[i] = (Range[359]+Range[1])/2;
        }
      }
      else if (i == 359)
      {
        if ((Range[359]-Range[1]) > 20)
        {
          Range[i] = Range[1];
        }
        else
        {
          Range[i] = (Range[0]+Range[358])/2;
        }
      }
      else
      {
        if ((Range[i-1]-Range[i+1]) > 20)
        {
          Range[i] = Range[i+1];
        }
        else
        {
          Range[i] = (Range[i-1]+Range[i+1])/2;
        }
      }
    }
  }
}


int Controller::GapDetect(double Range[359])
{
  int gap = 0;
  int gapStart = 0;
  int gapEnd = 0;
  int largestGap = 0;
  int largestGapStart = 0;
  int largestGapEnd = 0;
  int index;
  int adjacentIndex;
  for (int i = 0; i < 719; ++i)
  {
    // Check if the range is +- 20cm of the last reading 
    index = i%360;
    adjacentIndex = ((i + 1)%360);

    // if ((Range[index] < (Range[adjacentIndex] + 200)) && (Range[index] > (Range[adjacentIndex] - 200)))
    if(abs(Range[index] - Range[adjacentIndex]) > 200)
    {
      gap = gap + 1;
      if (gap > largestGap )
      {
        largestGap = gap;
        largestGapStart = gapStart%360; 
        largestGapEnd = (i+1)%360; 
      }
      gapEnd = i;
    }
    else
    {
      gap = 1;
      gapStart = i+1; 
    }
  }

  int turnTo = ((largestGap/2) + largestGapStart)%359;

  return turnTo; 
}
