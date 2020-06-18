/*
  IRSensor.h - A header file for IRSensors class for an arduino based robot.
  Group 2: Freeman Porten, Lachlan Barnes, Jake Olliff, Calvin Lee

  IRSensor: a class that interfaces the physical IR sensor with the rest of the arduino code.
  It handles the conversion of analog signal to a distance value in mm using individual sensor calibrations
*/
#ifndef Phototransistors
#define Phototransistors_h

#include "Arduino.h"

class Phototransistors
{
public:
  Phototransistors(uint8_t pin1, uint8_t pin2); // Constructor for this class
  bool FireDetected(int threshold);

private:
  uint8_t ptPin1Active;
  uint8_t ptPin2Active;
  uint8_t ptPin1Ambient;
  uint8_t ptPin2Ambient;
};

#endif
