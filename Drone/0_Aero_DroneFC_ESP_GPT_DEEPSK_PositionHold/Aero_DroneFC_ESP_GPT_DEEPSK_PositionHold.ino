#include <Wire.h>
#include <ESP32Servo.h>
#include <ESP32_ppm.h>
#include <driver/ledc.h>
#include <BluetoothSerial.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <WiFiUdp.h>
#include <WiFi.h>
#include <WiFiManager.h>
//#include <ESPAsyncWebServer.h>
//#include <AsyncTCP.h>
#include <Adafruit_BMP280.h>

//Define GPS Setup
#define GPS_RX_PIN 18
#define GPS_TX_PIN 17
#define GPS_BAUD 9600

HardwareSerial gpsSerial(2);  //Serial2
TinyGPSPlus gps;

//Position Hold Global Variable
bool gpsHoldMode = false;
double targetLat = 0;
double targetLon = 0;
float gpsSpeed = 0;
bool gpsValid = false;

//Position PID (Outer: m/s velocity setpoint)
float PPos = 0.5f;   //P variable for position to velocity
float IPos = 0.1f;   //I variable
float DPos = 0.05f;  //Derivative for damping
float PrevErrorPosX = 0;
float PrevErrorPosY = 0;
float ItermPosX = 0;
float ItermPosY = 0;
float DesiredVelX = 0;
float DesiredVelY = 0;

//Velocity PID (Middle: m/s to angle setpoint)
float PVel = 1.0f;  //P variable for position to velocity
float IVel = 0.2f;  //I variable
float DVel = 0.1f;  //Derivative for damping
float PrevErrorVelX = 0;
float PrevErrorVelY = 0;
float ItermVelX = 0;
float ItermVelY = 0;
float DesiredAngleX = 0;
float DesiredAngleY = 0;

// Define min and max PPM values for ESC calibration
#define ESC_PPM_MIN 1148  // Change this to your ESC's minimum PPM value
#define ESC_PPM_MAX 1832  // Change this to your ESC's maximum PPM value
#define ESC_PPM_MIDDLE 1488

struct EscCal {
  uint16_t min_us;
  uint16_t max_us;
  float scale;  // multiplier to map generic PPM → real µs
};
EscCal esc[4] = {
  { 1148, 1832, 1.00f },  // EMAX #1
  { 1148, 1832, 1.00f },  // EMAX #2
  { 1148, 1832, 1.00f },  // EMAX #3
  { 1120, 1880, 0.78 }   // Cyclone – measured values (example)
};

#define FILTER_SIZE 10

int motorIndex = 0;

// Define the number of channels and PPM pin
#define CHANNEL_NUMBER 10
#define PPM_PIN 33

#include <Adafruit_BMP280.h>

// BMP280 Setup
Adafruit_BMP280 bmp;
float altitude = 0.0f;
float targetAltitude = 0.0f;
bool altitudeHoldMode = false;
float groundPressure;

// Altitude PID (Outer: meters → velocity)
float PAlt = 1.2f;  // Position → vertical speed
float IAlt = 0.3f;
float DAlt = 0.1f;
float PrevErrorAlt = 0;
float ItermAlt = 0;
float DesiredVerticalSpeed = 0;  // m/s

// Vertical Speed PID (Middle: m/s → throttle offset)
float PVert = 8.0f;  // Speed → throttle %
float IVert = 1.5f;
float DVert = 0.8f;
float PrevErrorVert = 0;
float ItermVert = 0;
float throttleOffset = 0;  // Added to InputThrottle

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
float PRateRoll = 0.6;  //0.6 - Default - 1 (tested)
float PRatePitch = PRateRoll;
float PRateYaw = 2;     //2 - 2

float IRateRoll = 3.5;  //3.5 - 2 (tested)
float IRatePitch = IRateRoll;
float IRateYaw = 12;     //12 - 12

float DRateRoll = 0.015;  //0.03 - 0.05 )tested
float DRatePitch = DRateRoll;
float DRateYaw = 0.0;  //0
*/

float PRateRoll = 0.75f;
float PRatePitch = 0.75f;
float PRateYaw = 1.8f;

float IRateRoll = 2.2f;
float IRatePitch = 2.2f;
float IRateYaw = 12.0f;

float DRateRoll = 0.018f;
float DRatePitch = 0.018f;
float DRateYaw = 0.0f;

float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// ESP32 PWM properties
const int PWM_Frequency = 50;
const int PWM_Resolution = 12;
const int MAX_PWM_VALUE = (1 << PWM_Resolution) - 1;

// Motor PWM pins (change these according to your ESP32 pinout)
#define MOTOR_PIN_1 16  //Changed
#define MOTOR_PIN_2 17  //Changed                                                                                                     0
#define MOTOR_PIN_3 21  //Changed
#define MOTOR_PIN_4 13

// Battery pin (change according to your setup)
const int BATTERY_PIN = 12;
//const int CURRENT_PIN = 19;

// LED pin
//const int LED_PIN = 2;

// Servo objects for motor control
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

ppmReader myPpmTx;  //PPM TX writer object
int* ppmArray;

int throttle_ppm;
int roll_ppm;
int pitch_ppm;
int yaw_ppm;
int arming_esc;
int angle_mode;

int lastThrottle = 1000;  // keep global

// ---------- Throttle ramp globals ----------
int currentThrottle = 1000;  // the current throttle value actually sent
int targetThrottle = 1000;   // desired target set by receiver / stick
bool rampActive = false;

unsigned long lastRampMillis = 0;  // timing helper
unsigned long rampIntervalMs = 5;  // ms between increments (will be computed if using totalRampTime)
int rampStep = 1;                  // how many µs to change per interval (1 = count every number)

int rampStartValue = 1000;
int rampEndValue = 1000;
unsigned long rampStartMillis = 0;
unsigned long rampTotalTimeMs = 0;  // if >0 we'll compute interval to finish in this time

int updateThrottleRamp() {
  static int lastSmoothedThrottle = 1000;
  const int rampStep = 5;  // Adjust for responsiveness

  if (throttle_ppm > lastSmoothedThrottle + rampStep) {
    lastSmoothedThrottle += rampStep;
  } else if (throttle_ppm < lastSmoothedThrottle - rampStep) {
    lastSmoothedThrottle -= rampStep;
  } else {
    lastSmoothedThrottle = throttle_ppm;
  }

  return constrain(lastSmoothedThrottle, ESC_PPM_MIN, ESC_PPM_MAX);
}

void battery_voltage(void) {
  Voltage = ((float)analogRead(BATTERY_PIN) / 152.5);
  //Current = (float)analogRead(CURRENT_PIN) * 0.089;
}

void read_receiver() {
  // For PPM sum receiver
  if (myPpmTx.newFrame()) {
    //throttle_ppm = 0.8 * throttle_ppm + 0.2 * ppmArray[3];
    throttle_ppm = ppmArray[4] - 400;
    roll_ppm = ppmArray[3] - 400;
    pitch_ppm = ppmArray[2] - 400;
    yaw_ppm = ppmArray[5] - 400;
    arming_esc = ppmArray[6] - 400;
    angle_mode = ppmArray[7] - 400;
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

  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  //Adding Low Pass Filter
  filteredRoll = GYRO_LPF * filteredRoll + (1 - GYRO_LPF) * RateRoll;
  filteredPitch = GYRO_LPF * filteredPitch + (1 - GYRO_LPF) * RatePitch;
  filteredYaw = GYRO_LPF * filteredYaw + (1 - GYRO_LPF) * RateYaw;

  RateRoll = filteredRoll;
  RatePitch = filteredPitch;
  RateYaw = filteredYaw;
}


// Position PID (similar to your rate, but for meters)
void pid_equation_pos(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;
  Iterm = constrain(Iterm, -1.0f, 1.0f);  // Limit velocity integral

  float Dterm = D * (Error - PrevError) / 0.004;
  float PIDOutput = Pterm + Iterm + Dterm;
  PIDOutput = constrain(PIDOutput, -2.0f, 2.0f);  // Max 2 m/s

  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

// Velocity PID (output in rad/s for angle)
void pid_equation_vel(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;
  Iterm = constrain(Iterm, -0.5f, 0.5f);

  float Dterm = D * (Error - PrevError) / 0.004;
  float PIDOutput = Pterm + Iterm + Dterm;
  PIDOutput = constrain(PIDOutput, -1.0f, 1.0f);  // Max angle rate

  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}


void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;

  if (Iterm > 400) Iterm = 400;
  else if (Iterm < -400) Iterm = -400;

  float Dterm = D * (Error - PrevError) / 0.004;
  float PIDOutput = Pterm + Iterm + Dterm;

  if (PIDOutput > 400) PIDOutput = 400;
  else if (PIDOutput < -400) PIDOutput = -400;

  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void reset_pid(void) {
  PrevErrorPosX = PrevErrorPosY = 0;
  ItermPosX = ItermPosY = 0;
  PrevErrorVelX = PrevErrorVelY = 0;
  ItermVelX = ItermVelY = 0;
}

// Simple throttle smoothing without oscillations
int smoothThrottle(int targetThrottle) {
  static int currentThrottle = 1000;
  const int maxStep = 10;  // Maximum change per cycle

  // If throttle is at minimum or we're arming, respond immediately
  if (targetThrottle <= ESC_PPM_MIN + 50) {
    currentThrottle = targetThrottle;
    return currentThrottle;
  }

  // Smooth transition for other throttle values
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

float linearToThrust(float us) {
  float t = (us - ESC_PPM_MIN) / float(ESC_PPM_MAX - ESC_PPM_MIN);  // 0-1
  return t * t;                                                     // quadratic
}
float thrustToUs(float thrust, const EscCal& e) {
  float t = sqrtf(thrust);  // inverse
  return e.min_us + t * (e.max_us - e.min_us);
}

void setup() {
  Serial.begin(115200);

  // After PID mixer, before constrain
  float throttleNorm = (InputThrottle - ESC_PPM_MIN) / (float)(ESC_PPM_MAX - ESC_PPM_MIN);

  float frontReduction = 0.15f * throttleNorm;
  float rearBoost = 0.08f * throttleNorm;

  MotorInput1 *= (1.0f - frontReduction);
  MotorInput2 *= (1.0f - frontReduction);
  MotorInput3 *= (1.0f + rearBoost);
  MotorInput4 *= (1.0f + rearBoost);

  // Then constrain
  MotorInput1 = constrain(MotorInput1, ESC_PPM_MIN, ESC_PPM_MAX);
  MotorInput2 = constrain(MotorInput2, ESC_PPM_MIN, ESC_PPM_MAX);
  MotorInput3 = constrain(MotorInput3, ESC_PPM_MIN, ESC_PPM_MAX);
  MotorInput4 = constrain(MotorInput4, ESC_PPM_MIN, ESC_PPM_MAX);

  // Initialize motors with Servo library (simpler approach)
  motor1.attach(MOTOR_PIN_1, ESC_PPM_MIN, ESC_PPM_MAX);
  motor2.attach(MOTOR_PIN_2, ESC_PPM_MIN, ESC_PPM_MAX);
  motor3.attach(MOTOR_PIN_3, ESC_PPM_MIN, ESC_PPM_MAX);
  motor4.attach(MOTOR_PIN_4, ESC_PPM_MIN, ESC_PPM_MAX);

  motor1.setPeriodHertz(PWM_Frequency);
  motor2.setPeriodHertz(PWM_Frequency);
  motor3.setPeriodHertz(PWM_Frequency);
  motor4.setPeriodHertz(PWM_Frequency);

  // Write minimum signal to initialize ESCs
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
  Serial.println("ESC_Armed and Ready!");
  Serial.println("  моей семьи, моей родины ");
  //delay(2000);

  //Initialize PPM Pin
  ppmArray = myPpmTx.begin(PPM_PIN);
  myPpmTx.start();

  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Initialize BMP280
  /*
  if (!bmp.begin(0x76)) {  // Try 0x76 first, then 0x77
    if (!bmp.begin(0x77)) {
      Serial.println("BMP280 not found! Check wiring.");
      while (1)
        ;
    }
  }
  Serial.println("BMP280 initialized");

  // Take initial altitude as reference
  delay(100);
  groundPressure = bmp.readPressure();
  Serial.print("Ground Pressure: ");
  Serial.println(groundPressure);
  */
  // Initialize GPS
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS Initialized");

  // Gyro calibration
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }

  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;

  // Setup receiver pins
  for (int i = 0; i < min(CHANNEL_NUMBER, 4); i++) {
    pinMode(PPM_PIN + i, INPUT);
  }

  battery_voltage();

  if (Voltage > 9.0) {
    //digitalWrite(LED_PIN, LOW);
    BatteryAtStart = BatteryDefault;
  } else if (Voltage < 12.6) {
    BatteryAtStart = 30 / 100 * BatteryDefault;
  } else {
    //digitalWrite(LED_PIN, LOW);
    Serial.println("Battery LOW!");
    BatteryAtStart = (82 * Voltage - 580) / 100 * BatteryDefault;
  }
  /**/
  // Wait for throttle to be in low position
  bool throttle_ok = false;
  while (!throttle_ok) {
    read_receiver();
    Serial.println("PPM Receieved!");

    if (throttle_ppm < ESC_PPM_MIN) {
      motor1.writeMicroseconds(ESC_PPM_MIN);
      motor2.writeMicroseconds(ESC_PPM_MIN);
      motor3.writeMicroseconds(ESC_PPM_MIN);
      motor4.writeMicroseconds(ESC_PPM_MIN);  //Cyclone

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

  // === READ ALTITUDE ===
  /*
  static uint32_t lastAltTime = 0;
  if (micros() - lastAltTime > 20000) {  // 50 Hz update
    lastAltTime = micros();

    // Convert pressure to altitude (relative to ground)
    float pressure = bmp.readPressure();
    altitude = bmp.readAltitude(groundPressure);  // Returns meters above ground

    // Enable Altitude Hold via AUX (reuse channel 7 or add new)
    if (angle_mode > 1600 && gpsValid) {  // Use same switch as GPS hold
      altitudeHoldMode = true;
      if (targetAltitude == 0) {
        targetAltitude = altitude;  // Lock current height
        Serial.print("Altitude Hold ENGAGED at ");
        Serial.print(targetAltitude);
        Serial.println("m");
      }
    } else {
      altitudeHoldMode = false;
      targetAltitude = 0;
      throttleOffset = 0;
    }

    if (altitudeHoldMode) {
      // Outer: Altitude error → vertical speed setpoint
      float errorAlt = targetAltitude - altitude;

      // PID for altitude
      float Pterm = PAlt * errorAlt;
      ItermAlt += IAlt * errorAlt * 0.02f;
      ItermAlt = constrain(ItermAlt, -5.0f, 5.0f);
      float Dterm = DAlt * (errorAlt - PrevErrorAlt) / 0.02f;
      DesiredVerticalSpeed = Pterm + ItermAlt + Dterm;
      DesiredVerticalSpeed = constrain(DesiredVerticalSpeed, -1.5f, 1.5f);  // Max ±1.5 m/s
      PrevErrorAlt = errorAlt;

      // Middle: Vertical speed error → throttle offset
      float currentVertSpeed = 0;  // Optional: integrate accel Z later
      float errorVert = DesiredVerticalSpeed - currentVertSpeed;

      float PtermV = PVert * errorVert;
      ItermVert += IVert * errorVert * 0.02f;
      ItermVert = constrain(ItermVert, -100.0f, 100.0f);
      float DtermV = DVert * (errorVert - PrevErrorVert) / 0.02f;
      throttleOffset = PtermV + ItermVert + DtermV;
      throttleOffset = constrain(throttleOffset, -200.0f, 200.0f);  // ±200 µs
      PrevErrorVert = errorVert;
    } else {
      ItermAlt = 0;
      ItermVert = 0;
      throttleOffset = 0;
    }
  }*/

  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  read_receiver();

  /* */
  InputThrottle = smoothThrottle(throttle_ppm);

  DesiredRateRoll = 0.15 * (roll_ppm - 1500);    //Roll
  DesiredRatePitch = 0.15 * (pitch_ppm - 1500);  //Pitch
  DesiredRateYaw = 0.15 * (yaw_ppm - 1500);      //Yaw

  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        gpsValid = true;
        double currentLat = gps.location.lat();
        double currentLon = gps.location.lng();
        gpsSpeed = gps.speed.mps();  // Ground speed (m/s, scalar for now)

        // Set target on first valid fix (or arm)
        if (targetLat == 0 && targetLon == 0 && arming_esc > 1500) {
          targetLat = currentLat;
          targetLon = currentLon;
          Serial.print("GPS Hold Target: ");
          Serial.print(targetLat, 6);
          Serial.print(", ");
          Serial.println(targetLon, 6);
        }

        // Enable GPS Hold via AUX (e.g., channel 8 > 1500)
        if (angle_mode > 1500 && gpsValid) {  // Reuse angle_mode as AUX for mode switch
          gpsHoldMode = true;
        } else {
          gpsHoldMode = false;
          DesiredVelX = DesiredVelY = 0;  // No hold
        }

        if (gpsHoldMode) {
          // Calculate position errors (simple Haversine approx for small distances)
          float deltaLat = (currentLat - targetLat) * 111139;                              // Degrees to meters (lat)
          float deltaLon = (currentLon - targetLon) * 111139 * cos(targetLat * PI / 180);  // Lon adjustment

          // Outer Position PID: Error (m) → Velocity setpoint (m/s)
          float ErrorPosX = deltaLon;  // East-West
          float ErrorPosY = deltaLat;  // North-South

          pid_equation_pos(ErrorPosX, PPos, IPos, DPos, PrevErrorPosX, ItermPosX);  // X (roll direction)
          DesiredVelX = PIDReturn[0];                                               // m/s east
          PrevErrorPosX = PIDReturn[1];
          ItermPosX = PIDReturn[2];

          pid_equation_pos(ErrorPosY, PPos, IPos, DPos, PrevErrorPosY, ItermPosY);  // Y (pitch direction)
          DesiredVelY = PIDReturn[0];                                               // m/s north
          PrevErrorPosY = PIDReturn[1];
          ItermPosY = PIDReturn[2];

          // Middle Velocity PID: Velocity error → Angle setpoint (degrees)
          // Approx: vel from GPS speed & heading (gps.course.deg() for direction)
          float currentVelX = gpsSpeed * sin(gps.course.deg() * PI / 180);  // East component
          float currentVelY = gpsSpeed * cos(gps.course.deg() * PI / 180);  // North component

          float ErrorVelX = DesiredVelX - currentVelX;
          float ErrorVelY = DesiredVelY - currentVelY;

          pid_equation_vel(ErrorVelX, PVel, IVel, DVel, PrevErrorVelX, ItermVelX);
          DesiredAngleX = PIDReturn[0] * 57.3f;  // rad to deg (roll)
          PrevErrorVelX = PIDReturn[1];
          ItermVelX = PIDReturn[2];

          pid_equation_vel(ErrorVelY, PVel, IVel, DVel, PrevErrorVelY, ItermVelY);
          DesiredAngleY = PIDReturn[0] * 57.3f;  // rad to deg (pitch)
          PrevErrorVelY = PIDReturn[1];
          ItermVelY = PIDReturn[2];

          // Feed to your existing angle PID (replace DesiredRateRoll/Pitch)
          DesiredRateRoll = DesiredAngleX * 0.15f;  // Scale to match your stick scaling
          DesiredRatePitch = DesiredAngleY * 0.15f;
        }
      } else {
        gpsValid = false;
        Serial.println("Waiting for GPS fix...");
      }
    }
  }

 
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
 
  MotorInput1 = InputThrottle - InputRoll - InputPitch - InputYaw;
  MotorInput2 = InputThrottle - InputRoll + InputPitch + InputYaw;
  MotorInput3 = InputThrottle + InputRoll + InputPitch - InputYaw;
  MotorInput4 = InputThrottle + InputRoll - InputPitch + InputYaw;

  float rollTrim = +0.06f;
  MotorInput1 += rollTrim * 100;
  MotorInput2 += rollTrim * 100;
  MotorInput3 += rollTrim * 100;
  MotorInput4 += rollTrim * 100;


  if (InputThrottle <= ESC_PPM_MIN + 10) {
    motor1.writeMicroseconds(esc[0].min_us);
    motor2.writeMicroseconds(esc[1].min_us);
    motor3.writeMicroseconds(esc[2].min_us);
    motor4.writeMicroseconds(esc[3].min_us);
    reset_pid();
    // skip motor write below
  } else {
    // 4. Apply per-ESC calibration (Fix #2)
    uint16_t generic[4] = {
      constrain(MotorInput1, ESC_PPM_MIN, ESC_PPM_MAX),
      constrain(MotorInput2, ESC_PPM_MIN, ESC_PPM_MAX),
      constrain(MotorInput3, ESC_PPM_MIN, ESC_PPM_MAX),
      constrain(MotorInput4, ESC_PPM_MIN, ESC_PPM_MAX)
    };

    for (int i = 0; i < 4; i++) {
      float norm = (generic[i] - ESC_PPM_MIN) / float(ESC_PPM_MAX - ESC_PPM_MIN);
      uint16_t us = esc[i].min_us + (uint16_t)(norm * (esc[i].max_us - esc[i].min_us));
      us = (uint16_t)(us * esc[i].scale);

      switch (i) {
        case 0: motor1.writeMicroseconds(us); break;
        case 1: motor2.writeMicroseconds(us); break;
        case 2: motor3.writeMicroseconds(us); break;
        case 3: motor4.writeMicroseconds(us); break;
      }
    }
  }



  // Write to motors using Servo library (much simpler)
  /*
  motor1.writeMicroseconds(MotorInput1);
  motor2.writeMicroseconds(MotorInput2);
  motor3.writeMicroseconds(MotorInput3);
  motor4.writeMicroseconds(MotorInput4); 
  */

  battery_voltage();
  CurrentConsumed = Current * 1000 * 0.004 / 3600 + CurrentConsumed;
  BatteryRemaining = (BatteryAtStart - CurrentConsumed) / BatteryDefault * 100;

  if (BatteryRemaining <= 30) {
    Serial.println("Battery is Low!");
    //digitalWrite(LED_PIN, HIGH);

  }  //else digitalWrite(LED_PIN, LOW);


  power_print();

  Serial.println();

  while (micros() - LoopTimer < 4000)
    ;
  LoopTimer = micros();
}


void power_print() {
  Serial.print(MotorInput1);  // Tuned
  Serial.print(",");
  Serial.print(MotorInput2);
  Serial.print(",");
  Serial.print(MotorInput3);  // Tuned
  Serial.print(",");
  Serial.print(MotorInput4);
  Serial.print(",");
  Serial.print("   ");
  //Serial.print(Voltage); // Tuned
  //Serial.print(",");
  Serial.print(RatePitch);
  Serial.print(",");
  Serial.print(RateYaw);  // Tuned
  Serial.print(",");
  Serial.print(RateRoll);
  Serial.print(",");
  Serial.print(yaw_ppm);
}
