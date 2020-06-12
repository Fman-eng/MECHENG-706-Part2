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
  double IRDiffTol = 20; //Difference between IR values
  double IRAvgTol = 200 ; //Average between IR values
  bool finished = ((abs(frontIR - backIR) < IRDiffTol) & (((frontIR+backIR)/2)<IRAvgTol)); 
  if(abs(frontIR - backIR) > IRDiffTol){
    Serial.println("Failing the difference condition");
    Serial.println(abs(frontIR - backIR));
  }
  if(((frontIR+backIR)/2)>IRAvgTol){
    Serial.println("Failing the average condition");
    Serial.println(((frontIR+backIR)/2));
  }
  out[0] = 0;
  out[1] = 0;
  out[2] = -25; // Turning speed initially, CCW
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

/**
 * Scan distances around the robot 
 * 
 * While the robot is rotate take the angle the robot is at 
 * and the distance of the object ahead of the robot and 
 * store the distance value in an array using the angle as the 
 * array index
 */

void Controller::GapScan(double Range[359], int angle, double distance)
{
  Range[angle] = distance;
}

/**
 * Fill in any hole in the gap array
 * 
 * Checks for zero values in the array and fills it in depending
 * on what values are in the adjacent indexes. The array may need to 
 * be passed through a few times to remove all the holes.
 */

void Controller::GapFill(double Range[359])
{
  // For each value in the array
  for (int i = 0; i < 359; ++i)
  {
    // Check if it is zero
    if (Range[i] == 0)
    {
      // Check if it is a fringe case
      if (i == 0)
      {
        // Check if a change in surface occured 
        // If yes copy the value accross 
        if ((Range[359]-Range[1]) > 20)
        {
          Range[i] = Range[1];
        }
        // If not a change in surface take an average 
        else
        {
          Range[i] = (Range[359]+Range[1])/2;
        }
      }
      else if (i == 359)
      {
        // Check if a change in surface occured 
        // If yes copy the value accross
        if ((Range[359]-Range[1]) > 20)
        {
          Range[i] = Range[1];
        }
        // If not a change in surface take an average
        else
        {
          Range[i] = (Range[0]+Range[358])/2;
        }
      }
      else
      {
        // Check if a change in surface occured 
        // If yes copy the value accross
        if ((Range[i-1]-Range[i+1]) > 20)
        {
          Range[i] = Range[i+1];
        }
        // If not a change in surface take an average
        else
        {
          Range[i] = (Range[i-1]+Range[i+1])/2;
        }
      }
    }
  }
}

/**
 * Processes the array and finds the largest gap size returning
 * the best angle to turn at.
 */


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
    index = i%360;
    adjacentIndex = ((i + 1)%360);

    // Check if the range is +- 20cm of the last reading 
    if(abs(Range[index] - Range[adjacentIndex]) < 200)
    {
      // If we are still scanning the same surface increment the gap size
      gap = gap + 1;

      // If the gap is larger than the largest gap it now becomes the 
      // largest gap. 
      if (gap > largestGap )
      {
        largestGap = gap;
        largestGapStart = gapStart%360; 
        largestGapEnd = (i+1)%360; 
      }
      gapEnd = i;
    }
    // If a new surface is detected reset the gap size
    else
    {
      gap = 1;
      gapStart = i+1; 
    }
  }

  // Calculate the angle in the middle of the largest gap
  int turnTo = ((largestGap/2) + largestGapStart)%360;

  return turnTo; 
}
