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
  IRSensor(uint8_t pin, int sensor); // Constructor for this class
  float getDistance();                 // This function returns the distance to the wall in mm
  int getSensorReading();            // This function returns one raw sensor reading
  float getAverage();

private:
  uint8_t _pin; //Pin number assigned to the sensor
  float IRValues[5] = {500,500,500,500,500};
  int firItr;
  int _sensor;  //Is this sensor1 or sensor2 (written on bottom of sensor module)
  int sensorReadings[5];
  
  float sensorCoefficients[4][4] = {{909, -0.02958, 282.9, -0.003827},
                                    {1065, -0.1442, 284.7, -0.01242},
                                    {898.9, -0.0263, 246.8, -0.003013},
                                    {1065, -0.1442, 284.7, -0.01242}};
  
  
                                    // {{909, -0.02958, 282.9, -0.003827},
                                    // {1065, -0.1442, 284.7, -0.01242},
                                    // {898.9, -0.0263, 246.8, -0.003013},
                                    // {948.5, -0.1262, 250.4, -0.01015}};
  
  
  
   //Buffer to hold five sensor readings
  //float sensorOneCoeffs[4] = {909, -0.02958, 282.9, -0.003827};    //Sensor1 model coefficients
  //float sensorThreeCoeffs[4] = {898.9, -0.0263, 246.8, -0.003013};   //Sensor2 model coefficients
  //float sensorTwoCoeffs[4] = {1065, -0.1442, 284.7, -0.01242};   //Sensor3 model coefficients
  //float sensorFourCoeffs[4] = {948.5, -0.1262, 250.4,  -0.01015};  //Sensor4 model coefficients
  int sum;                  //Sum variable for internal calculation
  int averageSensorReading; //Average of the five sensor readings in the sensorReadings buffer
  int calculatedDistance;   //Calculated distance returned to the programmer
};

#endif
