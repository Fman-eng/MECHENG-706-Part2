/*
  IRSensor.cpp - A header file for IRSensors class for an arduino based robot.
  Group 2: Freeman Porten, Lachlan Barnes, Jake Olliff, Calvin Lee

  IRSensor: a class that interfaces the physical IR sensor with the rest of the arduino code.
  It handles the conversion of analog signal to a distance value in mm using individual sensor calibrations
*/
#include "Arduino.h"
#include "Phototransistors.h"

/**
 * Constructor for the IRSensor class. 
 * 
 * Takes an arduino pin number, saves it to a private class variable and 
 * sets the pin direction on the arduino. Also takes a boolean value to distinguish the model values, true uses
 * calibration values of sensor one, false uses calibration values for sensor two.
 */
Phototransistors::Phototransistors(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4)
{
    this->ptPin1Active = pin1;
    this->ptPin2Active = pin2;
    this->ptPin1Ambient= pin3;
    this->ptPin2Ambient = pin4;
}

bool Phototransistors::FireDetected(int threshold){
  int activeReading1 = 0;
  int activeReading2 = 0;

  for(int i = 0; i < 10; i++){
    activeReading1 += analogRead(this->ptPin1Active);
    activeReading2 += analogRead(this->ptPin2Active);
  }
  
  activeReading1 = activeReading1/10;
  activeReading2 = activeReading2/10;
  
  int offset = 0;
  
  //Serial.print(activeReading1+offset);
  //Serial.print(',');
  //Serial.println(activeReading2);

  return ((activeReading1+activeReading2)/2 > threshold);
}
  /**
 * Get the IR sensor reading. 
 * 
 * getDistance takes no input arguments and returns the distance measured by the IR sensor in mm. Takes the average
 * value of five sensor readings as a simple method to slightly reduce noise. This function uses a two-term exponential
 * model to calculate the sensor readings
 */
