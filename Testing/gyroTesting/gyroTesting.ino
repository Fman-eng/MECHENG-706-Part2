#include "gyro.h"

int gyroPin = A5;               //Gyro is connected to analog pin 0
float ADCZero = 508.65;   //Gyro is zeroed at 2.485V
float gyroSensitivity = 0.007;  //Our example gyro is 7mV/deg/sec

gyro gyroscope(gyroPin,ADCZero,gyroSensitivity);


void setup() {
  Serial.begin(9600);
}

void loop() {
 Serial.println(gyroscope.gyroUpdate());

 if (Serial.available() > 0){
        gyroscope.gyroReset();
        Serial.read();
  }
  Serial.flush();
}
