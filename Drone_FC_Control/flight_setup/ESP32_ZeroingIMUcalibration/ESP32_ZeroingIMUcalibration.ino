/* Zeronig IMU - Gyroscope Calibration */

#include <Wire.h>

float RateRoll, RatePitch, RateYaw;



void setup() {
  Wire.beginTransmission(0x68);

  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

}

void loop() {
  // put your main code here, to run repeatedly:

}
