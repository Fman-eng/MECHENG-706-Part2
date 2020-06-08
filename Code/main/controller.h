/*
  Controller.h - Library for a Controller of a arduino based robot.
  Group 2: Freeman Porten, Lachlan Barnes, Jake Olliff, Calvin Lee

  Controller: a class that controls is responsible for the overal
  control systems of the robot. It's contain methods such as InitForWall
  and WallFollow which are the initilizaion and wall following programs
  that take sensor readings and return desired PID actions.
*/
#ifndef Controller_h
#define Controller_h
#include "Arduino.h"

class Controller
{
private:
public:
  Controller();
  bool InitForWall(double frontIR, double backIR, double out[3]);
  void WallFollow(double frontIR, double backIR, double targetDistance, double out[3]);
  void FrontDetect(double sonar, double targetDistance, double out[3]);
  void GapScan(double Range[360], int angle, double distance);
  void GapFill(double Range[360]);
  int GapDetect(double Range[720]);
  void GyroTurn(double, double, double*);

};
#endif
