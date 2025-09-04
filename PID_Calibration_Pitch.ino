/* --- THIS IS THE CODE TO TEST THE CALIBRATION FOR ESC's ---- */
/*  
  Command : Setup -> Motor Start spinning at a startup rate 

*/

#include "Wire.h"
#include "ESP32Servo.h"

//potentiometer - value
#define poten 39
float poten_rate;
float pwm_rate;

float longitude;
float latitude;

/* ---- GYROSCOPE VARIABLE ----*/
float RateRoll, RatePitch, RateYaw;  //Gyro Global Variables
//Calibration Variable
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

/* ---- ACCELEROMETER VARIABLE ----*/
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

/* ---- ACCELERATION AND VELOCITY VARIABLE ---- */
float AccZInertial;
float VelocityVertical;

float LoopTimer;

/* ---- KALMAN FILTER VARIABLE ---- */
float KalmanAngleRoll = 0;
float KalmanUncertaintyAgleRoll = 2 * 2;
float KalmanAnglePitch = 0;
float KalmanUncertaintyAglePitch = 2 * 2;

float Kalman1DOutput[] = { 0, 0 };

//Filter Constant
float alpha = 0.98;

/* ---- PID VARIABLE VALUES ---- */
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevErrorItermRateRoll, PrevErrorItermRatePitch, PrevErrorItermRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[] = { 0, 0, 0 };
float IntegralPitch;

//PID Constant Variable
float PRateRoll = 0.5;
float PRatePitch = PRateRoll;
float PRateYaw = 2;

float IRateRoll = 0;
float IRatePitch = IRateRoll;
float IRateYaw = 12;

float DRateRoll = 0;
float DRatePitch = DRateRoll;
float DRateYaw = 0;

float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

//Timing
unsigned long prevMillis = 0;
//float dt;

void Kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput,
               float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;

  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

#define signal_led 2

#define voltage_adc 36
float voltage;

#define min_thorttle 1148     //Min PPM Throttle
#define max_thorttle 1832     //Max PPM Throttle
#define center_throttle 1488  //Center PPM Throttle

#define arming 1832    //Arming PPM Signal
#define arm_time 2000  //ESC Arming Time

//Left ESC pin - A
#define esc_pin1 27  //Cw
#define esc_pin2 25  //CCW

//Right ESC pin - B
#define esc_pin3 4   //CW
#define esc_pin4 12  //CCW - Cyclone

//ESC Attach
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

double dt, last_time;
double rate, desired_rate, actual_rate;
float output_pid;

int incomingCom = 0;

void arming_signal() {
  Serial.println("ARMING ESC!");

  esc1.writeMicroseconds(arming);
  esc2.writeMicroseconds(arming);
  esc3.writeMicroseconds(arming);
  esc4.writeMicroseconds(arming);
  unsigned long now = millis();
  while (millis() < now + arm_time) {
    // wait for ESC to arm
  }
}

void calibrating() {
  Serial.println("Sending MAX throttle signal...");
  esc1.writeMicroseconds(max_thorttle);
  esc2.writeMicroseconds(max_thorttle);
  esc3.writeMicroseconds(max_thorttle);
  //esc4.writeMicroseconds(max_thorttle);
  delay(5000);
  Serial.println("Sending MIN throttle signal...");
  esc1.writeMicroseconds(min_thorttle);
  esc2.writeMicroseconds(min_thorttle);
  esc3.writeMicroseconds(min_thorttle);
  //esc4.writeMicroseconds(min_thorttle);
  delay(5000);
  Serial.println("Sending CENTER throttle signal...");
  esc1.writeMicroseconds(center_throttle);
  esc2.writeMicroseconds(center_throttle);
  esc3.writeMicroseconds(center_throttle);
  esc4.writeMicroseconds(center_throttle);
  delay(5000);

  Serial.println("ESC_ARMED!");
  Serial.println("ESC - Calibrated - Completed");
}

void soft_start_to_center() {
  int startPulse = min_thorttle;
  int endPulse = center_throttle;
  int stepSize = 2;    // µs per step (smaller = smoother, slower start)
  int delayTime = 10;  // ms between steps

  for (int pulse = startPulse; pulse <= endPulse; pulse += stepSize) {
    esc1.writeMicroseconds(pulse);
    esc2.writeMicroseconds(pulse);
    esc3.writeMicroseconds(pulse);
    esc4.writeMicroseconds(pulse);
    delay(delayTime);
  }
}


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

  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

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
  AccX = (float)AccXLSB / 4096 - 0.06;
  AccY = (float)AccYLSB / 4096 - 0.01;
  AccZ = (float)AccZLSB / 4096 - 0.02;

  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
}

void pitch_pid() {
  //DesiredRatePitch = map(pwm_rate, min_thorttle, max_thorttle, -75, 70);
  DesiredRatePitch = 0;  //For Hovering
  ErrorRatePitch = DesiredRatePitch - AnglePitch;

  if ( -3 < DesiredRatePitch < 3) {
    IntegralPitch = IntegralPitch + (IRatePitch * (ErrorRatePitch * dt));
  }

  InputPitch = (PRatePitch * ErrorRatePitch) + IntegralPitch + (DRatePitch * ErrorRatePitch / dt);
}

void setup() {
  /* ---- IMU CALIBRATION AND SETUP ----*/
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


  Serial.begin(115200);

  pinMode(poten, INPUT);
  pinMode(signal_led, OUTPUT);

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  esc1.setPeriodHertz(50);  // Standard 50hz servo
  esc2.setPeriodHertz(50);  // Standard 50hz servo
  esc3.setPeriodHertz(50);  // Standard 50hz servo
  esc4.setPeriodHertz(50);  // Standard 50hz servo

  esc1.attach(esc_pin1, min_thorttle, max_thorttle);
  esc2.attach(esc_pin2, min_thorttle, max_thorttle);
  esc3.attach(esc_pin3, min_thorttle, max_thorttle);
  esc4.attach(esc_pin4, min_thorttle, max_thorttle);

  arming_signal();
  //calibrating();
  delay(7000);
  Serial.println("ESC Armed and Ready!");
  digitalWrite(signal_led, HIGH);
  esc1.writeMicroseconds(min_thorttle);  //Emax
  esc2.writeMicroseconds(min_thorttle);  //Emax
  esc3.writeMicroseconds(min_thorttle);  //Cyclone
  esc4.writeMicroseconds(min_thorttle);  //Emax
  delay(8000);
  digitalWrite(signal_led, LOW);
  soft_start_to_center();

  prevMillis = millis();
}

void loop() {
  unsigned long now = millis();
  dt = (now - prevMillis) / 1000.0;
  prevMillis = now;

  gyro_signals();  //Calling Gyro Function
  AccZInertial = sin(AnglePitch * (3.142 / 180)) * AccX + cos(AnglePitch * (3.142 / 180)) * sin(AngleRoll * (3.142 / 180)) * AccY + cos(AnglePitch * (3.142 / 180)) * cos(AngleRoll * (3.142 / 180)) * AccZ;
  AccZInertial = (AccZInertial - 1) * 9.81 * 100;

  VelocityVertical = VelocityVertical + AccZInertial * 0.004;

  RateRoll -= RateCalibrationRoll;    //* dt
  RatePitch -= RateCalibrationPitch;  //* dt
  RateYaw -= RateCalibrationYaw;      //* dt

  Kalman_1d(KalmanAngleRoll, KalmanUncertaintyAgleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAglePitch = Kalman1DOutput[1];
  Kalman_1d(KalmanAnglePitch, KalmanUncertaintyAglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAglePitch = Kalman1DOutput[1];

  //Serial.print(AnglePitch);
  //Serial.print("  ");

  pitch_pid();
  //poten_pwm();
  pwm_run();
  data_send_array();

  //Serial.println();
  //delayMicroseconds(5);
}


void poten_pwm() {
  /********** MOTOR - Driving ************/
  poten_rate = analogRead(poten);
  //pwm_rate = map(poten_rate, 0, 4095, min_thorttle, max_thorttle);
  pwm_rate = 1488;

  esc1.writeMicroseconds(pwm_rate);  //Emax - 3
  esc2.writeMicroseconds(pwm_rate);  //Emax - 2
  esc3.writeMicroseconds(pwm_rate);  //Emax - 4
  esc4.writeMicroseconds(pwm_rate);  //Cyclone - 1

  /*Serial.print(poten_rate);
  Serial.print("  ");
  Serial.print(pwm_rate);
  Serial.print("  ");
  */

  data_send_array();
  Serial.println();
}

void pwm_run() {
  pwm_rate = 1488;
  InputPitch = constrain(InputPitch, -200, 200);

  MotorInput1 = constrain(pwm_rate - InputPitch, min_thorttle, max_thorttle);
  MotorInput2 = constrain(pwm_rate + InputPitch, min_thorttle, max_thorttle);
  MotorInput3 = constrain(pwm_rate - InputPitch, min_thorttle, max_thorttle);
  MotorInput4 = constrain(pwm_rate + InputPitch, min_thorttle, max_thorttle);

  int MIN_SPIN = min_thorttle + 50;  // just above stall point
  if (MotorInput1 < MIN_SPIN) MotorInput1 = MIN_SPIN;
  if (MotorInput2 < MIN_SPIN) MotorInput2 = MIN_SPIN;
  if (MotorInput3 < MIN_SPIN) MotorInput3 = MIN_SPIN;
  if (MotorInput4 < MIN_SPIN) MotorInput4 = MIN_SPIN;

  esc1.writeMicroseconds(MotorInput1);  //Emax - 3
  esc2.writeMicroseconds(MotorInput2);  //Emax - 2
  esc3.writeMicroseconds(MotorInput3);  //Cyclone - 1
  esc4.writeMicroseconds(MotorInput4);  //Emax - 4
}

void data_send_array() {
  String fcu_data = String(DesiredRatePitch) + "#" + String(AnglePitch) + "#" 
  + String(ErrorRatePitch) + "#" + String(MotorInput1) + "#" + String(MotorInput2) 
  + "#" + String(MotorInput3) + "#" + String(MotorInput4) + "#" + String (PRatePitch)
  + "#" + String(IntegralPitch) + "#" + String(DRatePitch) + "#" + String(ErrorRatePitch)
  + "#" + String(InputPitch) + "#" + String(longitude) + "#" + String(latitude)
  + "#" + String(AnglePitch) + "#" + String(AngleRoll) + "#" + String(RateYaw);
  Serial.print(fcu_data);

  Serial.println();
}
