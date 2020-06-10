/*
  SonarSensor.cpp - A source file for the sonar sensor class for an arduino based robot.
  Group 2: Freeman Porten, Lachlan Barnes, Jake Olliff, Calvin Lee
  
  SonarSensor: a class that interfaces the physical sonar sensor with the rest of the arduino code.
  It handles all of the calculations of distance so the user only needs to make one function call
  to get a distance reading from the sonar sensor
*/
#include "Arduino.h"
#include "SonarSensor.h"

/**
 * Constructor for the SonarSensor class
 * 
 * Constructor takes the arduino pin that trigger and echo
 * and the desired rotation (omega) and based on the robots
 * of the pin on the arduino.
 */
SonarSensor::SonarSensor(int triggerPin, int echoPin)
{
  _triggerPin = triggerPin;
  _echoPin = echoPin;

  pinMode(_triggerPin, OUTPUT);
  pinMode(_echoPin, INPUT);
}

/**
 * Get the sonar sensor value
 * 
 * getDistance function does not take any input arguments and returns the distance measured by the
 * sonar sensor in mm.
 */
float SonarSensor::getDistance()
{
  // Ensure trigger pin has been held low for at least 2 microseconds, then send 5 microsecond pulse
  // in the trigger pin
  digitalWrite(_triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(_triggerPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(_triggerPin, LOW);

  // Start timer and wait for the echo pin to go from high to low, timeout is set to 1m distance equivilent in microseconds
  float duration = pulseIn(_echoPin, HIGH, 58000);
  // If timeout has occurred return 1 meter, else return the calculated distance
  if (duration == 0)
  {
    return 10000;
  }
  else
  {
    return (float)(duration / 2.9 / 2);
  }
}
