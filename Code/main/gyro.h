/*
  gyro.h - Library for a the gyroscope of a arduino based robot.
  Group 2: Freeman Porten, Lachlan Barnes, Jake Olliff, Calvin Lee

  gyro is a class used to control and take readings from a single 
  axis, analog gyroscope.  
*/

#ifndef Gyro_h
#define Gyro_h

#include "Arduino.h"

class Gyro
{
public:
  Gyro(uint8_t pin, float ADCOffset, float sensitivity); // Constructor for this class
  float gyroUpdate();    //Must be called in the super loop
  void gyroReset();     //Resets the angle count

private:
    uint8_t _pin;
    float _ADCOffset;
    float _sensitivity;

    float gyroRates[10];
    int ringBufferIndex;
    float driftOffset = 0.86;

    long oldMillis;
    long newMillis;

    float currentAngle;
};

#endif
