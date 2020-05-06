/*
  IRSensor.h - A header file for IRSensors class for an arduino based robot.
  Group 2: Freeman Porten, Lachlan Barnes, Jake Olliff, Calvin Lee

  IRSensor: a class that interfaces the physical IR sensor with the rest of the arduino code.
  It handles the conversion of analog signal to a distance value in mm using individual sensor calibrations
*/
#ifndef IRSensor_h
#define IRSensor_h

#include "Arduino.h"

class IRSensor
{
public:
  IRSensor(uint8_t pin, bool sensorOne); // Constructor for this class
  int getDistance();                     // This function returns the distance to the wall in mm
  int getSensorReading();                // This function returns one raw sensor reading

private:
  uint8_t _pin;                                                  //Pin number assigned to the sensor
  bool _sensorOne;                                               //Is this sensor1 or sensor2 (written on bottom of sensor module)
  int sensorReadings[5];                                         //Buffer to hold five sensor readings
  float sensorOneCoeffs[4] = {909, -0.02958, 282.9, -0.003827};  //Sensor1 model coefficients
  float sensorTwoCoeffs[4] = {898.9, -0.0263, 246.8, -0.003013}; //Sensor2 model coefficients
  int sum;                                                       //Sum variable for internal calculation
  int averageSensorReading;                                      //Average of the five sensor readings in the sensorReadings buffer
  int calculatedDistance;                                        //Calculated distance returned to the programmer
};

#endif