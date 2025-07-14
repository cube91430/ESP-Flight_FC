// 1. Measuring Angle with Accelerometer and Gyroscope - Done (General)
// 2. Zeroing the IMU Values
// 3. Set the Kalman Filter to be used in the PID logic
// 4. Set up PID for thermal and hovering
// 5. Each Pitch, Roll, Hover, and Yaw has distinct PID varianle

#include "Wire.h"

const int led = 15;  //built in LED pin

/* ---- GYROSCOPE VARIABLE ----*/
float RateRoll, RatePitch, RateYaw;  //Gyro Global Variables
//Calibration Variable
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

/* ---- ACCELEROMETER VARIABLE ----*/
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

float LoopTimer;

//Filter Constant
float alpha = 0.98;

//Timing
unsigned long prevMillis = 0;
float dt;

void gyro_signals(void) {  //Start I2C command

  //Configuring the MPU-6050 Gyroscope
  Wire.beginTransmission(0x68);
  //Switch on the low pass filter
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  //Access Register Story Accelerometer Measurements
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  //Pulling Accelerometer Value
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);

  int16_t AccXLSB = Wire.read() << 8 |
    Wire.read();
  int16_t AccYLSB = Wire.read() << 8 |
    Wire.read();
  int16_t AccZLSB = Wire.read() << 8 |
    Wire.read();

  //Set the Sensitivity Scale Factor
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();

  //Access Register Story Gyro Measurements
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  //Receiving X, Y, and Z axis information
  Wire.requestFrom(0x68, 6);

  int16_t GyroX = Wire.read() << 8 | Wire.read();  //X - axis
  int16_t GyroY = Wire.read() << 8 | Wire.read();  //Y - axis
  int16_t GyroZ = Wire.read() << 8 | Wire.read();  //Z - axis

  //Convert measurement to degrees per second
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  //Convert Measurements to Physical Values
  AccX = (float)AccXLSB/4096 - 0.06;
  AccY = (float)AccYLSB/4096 - 0.01;
  AccZ = (float)AccZLSB/4096 - 0.02;

  AngleRoll = atan(AccY/sqrt(AccX*AccX + AccZ*AccZ)) * 1/(3.142/180);
  AnglePitch = -atan(AccX/sqrt(AccY*AccY + AccZ*AccZ)) * 1/(3.142/180);


}

void setup() {
  Serial.begin(115200);
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);

  //Setting I2C Clock Speed
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  //Turning ON the MPU-6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);

  Wire.endTransmission();

  //IMU Calibration - Gyroscope Zeroing
  for (RateCalibrationNumber = 0;     //The value all calibration Shold be
       RateCalibrationNumber < 2000;  //The amount it take to take the value from the IMU
       RateCalibrationNumber++) {     //
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  //Calculate Calibration Values
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;


  prevMillis = millis();
}

void loop() {
  unsigned long now = millis();
  dt = (now - prevMillis) / 1000.0;
  prevMillis = now;

  gyro_signals();  //Calling Gyro Function

  RateRoll -= RateCalibrationRoll;    //* dt
  RatePitch -= RateCalibrationPitch;  //* dt
  RateYaw -= RateCalibrationYaw;      //* dt

/*

  Serial.print("Roll Rate: ");
  Serial.print(RateRoll);
  Serial.print(" degree/s");

  Serial.print(", Pitch Rate: ");
  Serial.print(RatePitch);
  Serial.print(" degree/s");

  Serial.print(", Yaw Rate: ");
  Serial.print(RateYaw);
  Serial.print(" degree/s");

  Serial.print("Accelerometer X: ");
  Serial.print(AccX);
  Serial.print(", Accelerometer Y: ");
  Serial.print(AccY);
  Serial.print(", Accelerometer Z: ");
  Serial.print(AccZ);
*/
  Serial.print(AngleRoll);

  Serial.print("  ");

  Serial.print(AnglePitch);


  Serial.println();

  delayMicroseconds(5);
}
