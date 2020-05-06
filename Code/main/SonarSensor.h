
#ifndef SonarSensor_h
#define SonarSensor_h

#include "Arduino.h"

/*
  SonarSensor.h - A source file for the sonar sensor class for an arduino based robot.
  Group 2: Freeman Porten, Lachlan Barnes, Jake Olliff, Calvin Lee
  
  SonarSensor: a class that interfaces the physical sonar sensor with the rest of the arduino code.
  It handles all of the calculations of distance so the user only needs to make one function call
  to get a distance reading from the sonar sensor
*/
class SonarSensor
{
public:
  SonarSensor(int triggerPin, int echoPin); //Constructor for the SonarSensor class
  float getDistance();                      //function that returns a distance measured by the sonar sensor

private:
  int _triggerPin; //Stores which pin the trigger signal is connected to
  int _echoPin;    //Stores which pin the echo signal is connected to
};

#endif
