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

bool Phototransistors::FireDetected(){
  int offset = 50;
  int ambientReading = (analogRead(this->ptPin1Ambient) + analogRead(this->ptPin2Ambient)/2);
  int activeReading1 = analogRead(this->ptPin1Active);
  int activeReading2 = analogRead(this->ptPin2Active);
  delay(50);
  Serial.print(activeReading1+offset);
  Serial.print(',');
  Serial.println(activeReading2);
  //Serial.print(',');
  //Serial.println(ambientReading);

  if((activeReading1 > ptDetectionThreshold) && (abs(activeReading1-activeReading2) < ptDeviationThreshold)){
    return true;
  } else {
    return false;
  }
}
  /**
 * Get the IR sensor reading. 
 * 
 * getDistance takes no input arguments and returns the distance measured by the IR sensor in mm. Takes the average
 * value of five sensor readings as a simple method to slightly reduce noise. This function uses a two-term exponential
 * model to calculate the sensor readings
 */
