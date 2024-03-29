#include "Arduino.h"
#include "gyro.h"

gyro::gyro(uint8_t pin, float ADCOffset, float sensitivity)
{
    _pin = pin;
    pinMode(pin, INPUT);

    _ADCOffset = ADCOffset;
    _sensitivity = sensitivity;

    for(int i = 0; i < 10; i++){
        gyroRates[i] = 0;
    }

    currentAngle = 0.0;
    ringBufferIndex = 0;
}


float gyro::gyroUpdate(){
    newMillis = micros();
    float gyroRate = analogRead(_pin);

    gyroRate = gyroRate-_ADCOffset;

    gyroRates[ringBufferIndex] = gyroRate;

    float gyroAverage = 0;
    for(int i = 0; i<10; i++){
        gyroAverage += gyroRates[i];
    }
    gyroAverage /=10;

    gyroRate = (gyroAverage * 5)/1023;

    gyroRate /= _sensitivity;

    currentAngle += (gyroRate+driftOffset)/1000000*(newMillis-oldMillis);
    

    ringBufferIndex = (ringBufferIndex + 1)%10;
    oldMillis = newMillis;

    return currentAngle;

}



void gyro::gyroReset(){
    for(int i = 0; i < 10; i++){
        gyroRates[i] = 0;
    }

    ringBufferIndex = 0;
    currentAngle = 0.0;
}
