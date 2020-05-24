#ifndef gyro_h
#define gyro_h

#include "Arduino.h"

class gyro
{
public:
  gyro(uint8_t pin, float ADCOffset, float sensitivity); // Constructor for this class
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
