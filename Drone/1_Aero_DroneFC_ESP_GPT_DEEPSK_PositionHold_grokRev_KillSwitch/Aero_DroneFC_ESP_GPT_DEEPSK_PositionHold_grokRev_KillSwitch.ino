#include <Wire.h>
#include <ESP32Servo.h>
#include <ESP32_ppm.h>
#include <driver/ledc.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Adafruit_BMP280.h>
#include <math.h>  // For trig functions

// Define GPS Setup
#define GPS_RX_PIN 18
#define GPS_TX_PIN 17
#define GPS_BAUD 9600

HardwareSerial gpsSerial(2);  // Serial2
TinyGPSPlus gps;

// Position Hold Global Variable
bool gpsHoldMode = false;
double targetLat = 0;
double targetLon = 0;
float gpsSpeed = 0;
bool gpsValid = false;

// Position PID (Outer: m/s velocity setpoint)
float PPos = 0.5f;
float IPos = 0.1f;
float DPos = 0.05f;
float PrevErrorPosX = 0;
float PrevErrorPosY = 0;
float ItermPosX = 0;
float ItermPosY = 0;
float DesiredVelX = 0;
float DesiredVelY = 0;

// Velocity PID (Middle: m/s to angle setpoint)
float PVel = 1.0f;
float IVel = 0.2f;
float DVel = 0.1f;
float PrevErrorVelX = 0;
float PrevErrorVelY = 0;
float ItermVelX = 0;
float ItermVelY = 0;
float DesiredAngleX = 0;
float DesiredAngleY = 0;

// Define min and max PPM values for ESC calibration
#define ESC_PPM_MIN 1148
#define ESC_PPM_MAX 1832
#define ESC_PPM_MIDDLE 1488

struct EscCal {
  uint16_t min_us;
  uint16_t max_us;
  float scale;
};
EscCal esc[4] = {
  { 1148, 1832, 1.00f },  // EMAX #1
  { 1148, 1832, 1.00f },  // EMAX #2
  { 1148, 1832, 1.00f },  // EMAX #3
  { 1120, 1880, 0.88f }   // Cyclone
};

#define FILTER_SIZE 10

int motorIndex = 0;

// Define the number of channels and PPM pin
#define CHANNEL_NUMBER 10
#define PPM_PIN 33

// BMP280 Setup
Adafruit_BMP280 bmp;
float altitude = 0.0f;
float prevAltitude = 0.0f;  // For vert speed estimate
float targetAltitude = 0.0f;
bool altitudeHoldMode = false;
float groundPressure;

// Altitude PID (Outer: meters → velocity)
float PAlt = 1.2f;
float IAlt = 0.3f;
float DAlt = 0.1f;
float PrevErrorAlt = 0;
float ItermAlt = 0;
float DesiredVerticalSpeed = 0;

// Vertical Speed PID (Middle: m/s → throttle offset)
float PVert = 8.0f;
float IVert = 1.5f;
float DVert = 0.8f;
float PrevErrorVert = 0;
float ItermVert = 0;
float throttleOffset = 0;

float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
int RateCalibrationNumber;

float ReceiverValue[CHANNEL_NUMBER] = { 0 };
int ChannelNumber = 0;

float Voltage, Current, BatteryRemaining, BatteryAtStart;
float CurrentConsumed = 0;
float BatteryDefault = 4000;
uint32_t LoopTimer;

float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;

#define GYRO_LPF 0.7f

float filteredRoll = 0;
float filteredPitch = 0;
float filteredYaw = 0;

float PIDReturn[] = { 0, 0, 0 };

/*
float PRateRoll = 0.75f;
float PRatePitch = 0.75f;
float PRateYaw = 1.8f;

float IRateRoll = 2.2f;
float IRatePitch = 2.2f;
float IRateYaw = 12.0f;

float DRateRoll = 0.018f;
float DRatePitch = 0.018f;
float DRateYaw = 0.0f;

*/
float PRateRoll = 0.6f;
float PRatePitch = PRateRoll;
float PRateYaw = 2.0f;

float IRateRoll = 3.5f;
float IRatePitch = IRateRoll;
float IRateYaw = 12.0f;

float DRateRoll = 0.018f;
float DRatePitch = DRateRoll;
float DRateYaw = 0.0f;

float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// Motor PWM pins
#define MOTOR_PIN_1 16
#define MOTOR_PIN_2 17
#define MOTOR_PIN_3 21
#define MOTOR_PIN_4 13

// Battery pin
const int BATTERY_PIN = 12;

// Servo objects
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

ppmReader myPpmTx;
int* ppmArray;

int throttle_ppm;
int roll_ppm;
int pitch_ppm;
int yaw_ppm;
int arming_esc;
int angle_mode;
int kill_switch;
int kill_state;

int lastThrottle = 1000;

// Throttle smoothing
int smoothThrottle(int targetThrottle) {
  static int currentThrottle = 1000;
  const int maxStep = 10;

  if (targetThrottle <= ESC_PPM_MIN + 50) {
    currentThrottle = targetThrottle;
    return currentThrottle;
  }

  int difference = targetThrottle - currentThrottle;
  if (abs(difference) <= maxStep) {
    currentThrottle = targetThrottle;
  } else if (difference > 0) {
    currentThrottle += maxStep;
  } else {
    currentThrottle -= maxStep;
  }

  return currentThrottle;
}

void battery_voltage(void) {
  Voltage = ((float)analogRead(BATTERY_PIN) / 152.5f);
}

void read_receiver() {
  if (myPpmTx.newFrame()) {
    throttle_ppm = ppmArray[4] - 400;
    roll_ppm = ppmArray[3] - 400;
    pitch_ppm = ppmArray[2] - 400;
    pitch_ppm = map(pitch_ppm, 1000, 2000, 2000, 1000);
    yaw_ppm = ppmArray[5] - 400;

    arming_esc = ppmArray[6] - 400;
    kill_switch = ppmArray[7] - 400;
  }
}

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll = (float)GyroX / 65.5f;
  RatePitch = (float)GyroY / 65.5f;
  RateYaw = (float)GyroZ / 65.5f;

  // Low Pass Filter
  filteredRoll = GYRO_LPF * filteredRoll + (1 - GYRO_LPF) * RateRoll;
  filteredPitch = GYRO_LPF * filteredPitch + (1 - GYRO_LPF) * RatePitch;
  filteredYaw = GYRO_LPF * filteredYaw + (1 - GYRO_LPF) * RateYaw;

  RateRoll = filteredRoll;
  RatePitch = filteredPitch;
  RateYaw = filteredYaw;
}

// Rate PID Equation (missing in original)
void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.004f / 2.0f;

  if (Iterm > 400) Iterm = 400;
  else if (Iterm < -400) Iterm = -400;

  float Dterm = D * (Error - PrevError) / 0.004f;
  float PIDOutput = Pterm + Iterm + Dterm;

  if (PIDOutput > 400) PIDOutput = 400;
  else if (PIDOutput < -400) PIDOutput = -400;

  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

// Position PID
void pid_equation_pos(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.004f / 2.0f;
  Iterm = constrain(Iterm, -1.0f, 1.0f);

  float Dterm = D * (Error - PrevError) / 0.004f;
  float PIDOutput = Pterm + Iterm + Dterm;
  PIDOutput = constrain(PIDOutput, -2.0f, 2.0f);

  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

// Velocity PID
void pid_equation_vel(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.004f / 2.0f;
  Iterm = constrain(Iterm, -0.5f, 0.5f);

  float Dterm = D * (Error - PrevError) / 0.004f;
  float PIDOutput = Pterm + Iterm + Dterm;
  PIDOutput = constrain(PIDOutput, -1.0f, 1.0f);

  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void reset_pid(void) {
  PrevErrorRateRoll = 0;
  PrevErrorRatePitch = 0;
  PrevErrorRateYaw = 0;
  PrevItermRateRoll = 0;
  PrevItermRatePitch = 0;
  PrevItermRateYaw = 0;

  PrevErrorPosX = PrevErrorPosY = 0;
  ItermPosX = ItermPosY = 0;
  PrevErrorVelX = PrevErrorVelY = 0;
  ItermVelX = ItermVelY = 0;

  PrevErrorAlt = 0;
  ItermAlt = 0;
  PrevErrorVert = 0;
  ItermVert = 0;
  throttleOffset = 0;
}

void setup() {
  Serial.begin(115200);

  // Initialize motors
  motor1.attach(MOTOR_PIN_1, ESC_PPM_MIN, ESC_PPM_MAX);
  motor2.attach(MOTOR_PIN_2, ESC_PPM_MIN, ESC_PPM_MAX);
  motor3.attach(MOTOR_PIN_3, ESC_PPM_MIN, ESC_PPM_MAX);
  motor4.attach(MOTOR_PIN_4, ESC_PPM_MIN, ESC_PPM_MAX);

  motor1.setPeriodHertz(50);
  motor2.setPeriodHertz(50);
  motor3.setPeriodHertz(50);
  motor4.setPeriodHertz(50);

  // ESC Calibration (consolidated delays)
  motor1.writeMicroseconds(ESC_PPM_MAX);
  motor2.writeMicroseconds(ESC_PPM_MAX);
  motor3.writeMicroseconds(ESC_PPM_MAX);
  motor4.writeMicroseconds(ESC_PPM_MAX);
  delay(4000);

  motor1.writeMicroseconds(ESC_PPM_MIN);
  motor2.writeMicroseconds(ESC_PPM_MIN);
  motor3.writeMicroseconds(ESC_PPM_MIN);
  motor4.writeMicroseconds(ESC_PPM_MIN);
  delay(4000);
  Serial.println("Calibration Complete!");

  motor1.writeMicroseconds(ESC_PPM_MIN);
  motor2.writeMicroseconds(ESC_PPM_MIN);
  motor3.writeMicroseconds(ESC_PPM_MIN);
  motor4.writeMicroseconds(ESC_PPM_MIN);
  delay(2000);
  Serial.println("ESC Armed and Ready!");

  // PPM Init
  ppmArray = myPpmTx.begin(PPM_PIN);
  myPpmTx.start();

  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // BMP280 Init
  /*
  if (!bmp.begin(0x76) && !bmp.begin(0x77)) {
    Serial.println("BMP280 not found! Check wiring.");
    while (1)
      ;
  }
  Serial.println("BMP280 initialized");

  delay(100);
  groundPressure = bmp.readPressure();
  Serial.print("Ground Pressure: ");
  Serial.println(groundPressure);

  // GPS Init
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS Initialized");
  */

  // Gyro Calibration
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }

  RateCalibrationRoll /= 2000.0f;
  RateCalibrationPitch /= 2000.0f;
  RateCalibrationYaw /= 2000.0f;

  // Receiver Pins (PPM is on one pin, so this loop is unnecessary but harmless)
  for (int i = 0; i < min(CHANNEL_NUMBER, 4); i++) {
    pinMode(PPM_PIN + i, INPUT);
  }

  battery_voltage();

  if (Voltage > 9.0) {
    BatteryAtStart = BatteryDefault;
  } else if (Voltage < 12.6) {
    BatteryAtStart = 30.0f / 100.0f * BatteryDefault;
  } else {
    Serial.println("Battery LOW!");
    BatteryAtStart = (82.0f * Voltage - 580.0f) / 100.0f * BatteryDefault;
  }

  // Wait for low throttle
  bool throttle_ok = false;
  while (!throttle_ok) {
    read_receiver();
    if (throttle_ppm < ESC_PPM_MIN) {
      motor1.writeMicroseconds(ESC_PPM_MIN);
      motor2.writeMicroseconds(ESC_PPM_MIN);
      motor3.writeMicroseconds(ESC_PPM_MIN);
      motor4.writeMicroseconds(ESC_PPM_MIN);
      delay(1000);
      throttle_ok = true;
    }
    delay(4);
  }

  LoopTimer = micros();
}

void loop() {
  static uint32_t prevMicros = 0;
  uint32_t now = micros();

  if (now - prevMicros < 4000) return;
  prevMicros = now;

  gyro_signals();
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  read_receiver();

  InputThrottle = smoothThrottle(throttle_ppm);

  DesiredRateRoll = 0.15f * (roll_ppm - 1500);
  DesiredRatePitch = 0.15f * (pitch_ppm - 1500);
  DesiredRateYaw = 0.15f * (yaw_ppm - 1500);

  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  // Un-commented Rate PID
  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll = PIDReturn[0];
  PrevErrorRateRoll = PIDReturn[1];
  PrevItermRateRoll = PIDReturn[2];

  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch = PIDReturn[0];
  PrevErrorRatePitch = PIDReturn[1];
  PrevItermRatePitch = PIDReturn[2];

  pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
  InputYaw = PIDReturn[0];
  PrevErrorRateYaw = PIDReturn[1];
  PrevItermRateYaw = PIDReturn[2];

  // Apply throttle offset
  InputThrottle += throttleOffset;
  InputThrottle = constrain(InputThrottle, ESC_PPM_MIN + 50, ESC_PPM_MAX);

  // Altitude Hold (decoupled from gpsValid)
  static uint32_t lastAltTime = 0;
  /*
  if (now - lastAltTime > 20000) {  // 50 Hz
    lastAltTime = now;

    float pressure = bmp.readPressure();
    altitude = bmp.readAltitude(groundPressure);

    if (angle_mode > 1600) {  // Switch for alt hold
      altitudeHoldMode = true;
      if (targetAltitude == 0) {
        targetAltitude = altitude;
        Serial.print("Altitude Hold at ");
        Serial.println(targetAltitude);
      }
    } else {
      altitudeHoldMode = false;
      targetAltitude = 0;
      throttleOffset = 0;
    }

    if (altitudeHoldMode) {
      float errorAlt = targetAltitude - altitude;

      float Pterm = PAlt * errorAlt;
      ItermAlt += IAlt * errorAlt * 0.02f;
      ItermAlt = constrain(ItermAlt, -5.0f, 5.0f);
      float Dterm = DAlt * (errorAlt - PrevErrorAlt) / 0.02f;
      DesiredVerticalSpeed = Pterm + ItermAlt + Dterm;
      DesiredVerticalSpeed = constrain(DesiredVerticalSpeed, -1.5f, 1.5f);
      PrevErrorAlt = errorAlt;

      // Estimate vert speed (simple delta)
      float currentVertSpeed = (altitude - prevAltitude) / 0.02f;  // m/s
      prevAltitude = altitude;

      float errorVert = DesiredVerticalSpeed - currentVertSpeed;

      float PtermV = PVert * errorVert;
      ItermVert += IVert * errorVert * 0.02f;
      ItermVert = constrain(ItermVert, -100.0f, 100.0f);
      float DtermV = DVert * (errorVert - PrevErrorVert) / 0.02f;
      throttleOffset = PtermV + ItermVert + DtermV;
      throttleOffset = constrain(throttleOffset, -200.0f, 200.0f);
      PrevErrorVert = errorVert;
    } else {
      ItermAlt = 0;
      ItermVert = 0;
      throttleOffset = 0;
    }
  }*/

  if (kill_switch == 1000) {
    kill_state = 1;
  } else {
    kill_state = kill_state;
  } 

  if (kill_state == 1) {  //Kill_Switch
    motor1.writeMicroseconds(ESC_PPM_MIN);
    motor2.writeMicroseconds(ESC_PPM_MIN);
    motor3.writeMicroseconds(ESC_PPM_MIN);
    motor4.writeMicroseconds(ESC_PPM_MIN);

    Serial.println("KILL_SWITCH ON!!");
  } else if (kill_state == 0) {
    // Mixer
    MotorInput1 = 1.024f * (InputThrottle - InputRoll - InputPitch - InputYaw);
    MotorInput2 = 1.024f * (InputThrottle - InputRoll + InputPitch + InputYaw);
    MotorInput3 = 1.024f * (InputThrottle + InputRoll + InputPitch - InputYaw);
    MotorInput4 = 1.024f * (InputThrottle + InputRoll - InputPitch + InputYaw);

    // Constrain
    MotorInput1 = constrain(MotorInput1, ESC_PPM_MIN, ESC_PPM_MAX);
    MotorInput2 = constrain(MotorInput2, ESC_PPM_MIN, ESC_PPM_MAX);
    MotorInput3 = constrain(MotorInput3, ESC_PPM_MIN, ESC_PPM_MAX);
    MotorInput4 = constrain(MotorInput4, ESC_PPM_MIN, ESC_PPM_MAX);

    motor1.writeMicroseconds(MotorInput1);
    motor2.writeMicroseconds(MotorInput2);
    motor3.writeMicroseconds(MotorInput3);
    motor4.writeMicroseconds(MotorInput4);


    battery_voltage();
    CurrentConsumed = Current * 1000.0f * 0.004f / 3600.0f + CurrentConsumed;
    BatteryRemaining = (BatteryAtStart - CurrentConsumed) / BatteryDefault * 100.0f;

    if (BatteryRemaining <= 30) {
      Serial.println("Battery is Low!");
    }

    //power_print();
    exe_power_print();

    while (micros() - LoopTimer < 4000)
      ;
    LoopTimer = micros();
  }


}

void power_print() {
  //Serial.print(kill_state);
  //Serial.print("    ");
  Serial.print(MotorInput1);
  Serial.print(",");
  Serial.print(MotorInput2);
  Serial.print(",");
  Serial.print(MotorInput3);
  Serial.print(",");
  Serial.print(MotorInput4);
  Serial.print(",");
  Serial.print(throttle_ppm);
  Serial.print(",");
  Serial.print(yaw_ppm);
  Serial.print(",");
  Serial.print(pitch_ppm);
  Serial.print(",");
  Serial.print(roll_ppm);
  Serial.println();
}
void exe_power_print() {
  //Serial.print(kill_state);
  //Serial.print("    ");
  Serial.print(MotorInput1);
  Serial.print("  ");
  Serial.print(MotorInput2);
  Serial.print("  ");
  Serial.print(MotorInput3);
  Serial.print(",");
  Serial.print(MotorInput4);
  Serial.print(",");
  Serial.print(throttle_ppm);
  Serial.print(",");
  Serial.print(yaw_ppm);
  Serial.print(",");
  Serial.print(pitch_ppm);
  Serial.print(",");
  Serial.print(roll_ppm);
  Serial.println();
}