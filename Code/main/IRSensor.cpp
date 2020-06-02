/*
  IRSensor.cpp - A header file for IRSensors class for an arduino based robot.
  Group 2: Freeman Porten, Lachlan Barnes, Jake Olliff, Calvin Lee

  IRSensor: a class that interfaces the physical IR sensor with the rest of the arduino code.
  It handles the conversion of analog signal to a distance value in mm using individual sensor calibrations
*/
#include "Arduino.h"
#include "IRSensor.h"

/**
 * Constructor for the IRSensor class. 
 * 
 * Takes an arduino pin number, saves it to a private class variable and 
 * sets the pin direction on the arduino. Also takes a boolean value to distinguish the model values, true uses
 * calibration values of sensor one, false uses calibration values for sensor two.
 */
IRSensor::IRSensor(uint8_t pin, int sensor)
{
    _pin = pin;
    _sensor = sensor;
    pinMode(pin, INPUT);
}

/**
 * Get the IR sensor reading. 
 * 
 * getDistance takes no input arguments and returns the distance measured by the IR sensor in mm. Takes the average
 * value of five sensor readings as a simple method to slightly reduce noise. This function uses a two-term exponential
 * model to calculate the sensor readings
 */
int IRSensor::getDistance()
{
    // Take five readings from the sensor and average them
    sum = 0;

    for (int i = 0; i < 5; i++)
    {
        sensorReadings[i] = analogRead(_pin);
        sum += sensorReadings[i];
    }
    averageSensorReading = sum / 5;

    // Use the correct sensor calibration values to calculate the distance from the average sensor readings
    calculatedDistance = sensorCoefficients[_sensor][0] * exp(sensorCoefficients[_sensor][1] * averageSensorReading) + sensorCoefficients[_sensor][2] * exp(sensorCoefficients[_sensor][3] * averageSensorReading);
   

    // Return the calculated distance
    return calculatedDistance;
}

// getSensorReading takes no input arguments and returns the raw output of a single sensor
int IRSensor::getSensorReading()
{
    return analogRead(_pin);
}
