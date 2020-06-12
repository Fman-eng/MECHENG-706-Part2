/*
  gyro.cpp - Library for a the gyroscope of a arduino based robot.
  Group 2: Freeman Porten, Lachlan Barnes, Jake Olliff, Calvin Lee

  gyro is a class used to control and take readings from a single 
  axis, analog gyroscope.  
*/


#include "Arduino.h"
#include "Gyro.h"

//Gyro class constructor. Inputs the analog input pin the gyro is connected to, 
//a measured ADCOffset to account for drift and the sensitivity of the gyro chip
//measured in volts/degree
Gyro::Gyro(uint8_t pin, float ADCOffset, float sensitivity)
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

//gyroUpdate returns the current angle from the gyro. Must be called as often as possible
//as this method is responsible for integration of angular acceleration values. If not
//called often enough the gyro will become inaccurate and unpredictable.
float Gyro::gyroUpdate(){
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

    if(currentAngle < 0){
        currentAngle = 0;
    }
    
    return currentAngle;
}


//gyroReset resets the angle and the ring buffer inside the gyro class
//Call this when taking restarting rotation readings for the most accurate results
void Gyro::gyroReset(){
    for(int i = 0; i < 10; i++){
        gyroRates[i] = 0;
    }

    ringBufferIndex = 0;
    currentAngle = 0.0;
}
