#include "ESP32Servo.h"
#include "Wire.h"

const int led = 15;  //built in LED pin

float RateRoll, RatePitch, RateYaw;  //Gyro Global Variables

//Calibration Variable
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

uint32_t LoopTimer;  //Parameter containing the length of each control loop

/* ------------- PID PARAMETERS --------------- */
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[] = { 0, 0, 0 };

/* ------------- Roll, Pitch, and Yaw Rate --------------*/
//Values for PID Parameters
float PRateRoll = 0.6;
float PRatePitch = PRateRoll;
float PRateYaw = 2;
float IRateRoll = 3.5;
float IRatePitch = IRateRoll;
float IRateYaw = 12;
float DRateRoll = 0.03;
float DRatePitch = DRateRoll;
float DRateYaw = 0;

float MotorInput1, MotorInput2, MotorInput3, MotorInput4;  //Motor Input variable


#define min_thorttle 1080     //Min PPM Throttle
#define max_thorttle 1720     //Max PPM Throttle
#define center_throttle 1488  //Center PPM Throttle

#define arming 1720    //Arming PPM Signal
#define arm_time 2000  //ESC Arming Time

//Left ESC pin - A
#define esc_pin0 27  //CCW
#define esc_pin1 12  //CW

//Right ESC pin - B
#define esc_pin2 13  //CW
#define esc_pin3 14  //CCW

//Left PWM pin - B
#define pwm_pin0 27  //Servo A
#define pwm_pin1 19  //Servo B

//Right PWM pin - A
#define pwm_pin2 18  //Servo C
#define pwm_pin3 5   //Servo D

volatile int calib_button = 15;  //Calibration_Settings
#define led_pin 13               //Calibration LED Signal
#define poten 39                 //Analog Pin

//Servo
Servo pwm_servo0;
Servo pwm_servo1;
Servo pwm_servo2;
Servo pwm_servo3;

//ESC Attach
Servo esc0;
Servo esc1;
Servo esc2;
Servo esc3;

int last_pwm = 1488;

void arming_signal() {

  esc0.write(arming);
  unsigned long now = millis();
  while (millis() < now + arm_time) {
    // wait for ESC to arm
  }
}

void blink() {
  digitalWrite(led_pin, HIGH);
  delay(5);
  digitalWrite(led_pin, LOW);
  delay(5);
}

void calibrating() {
  Serial.println("Calibrating Time : 4m20s");
  Serial.println("Calibrating Min to Max PPM");
  int i;  //initial calibration signal
  //calibrating full to zero
  for (i = center_throttle; i <= max_thorttle; i++) {
    esc0.write(i);
    esc1.write(i);
    esc2.write(i);
    esc3.write(i);
    Serial.println(i);
    blink();
  }
  if (i == max_thorttle) {
    esc0.write(max_thorttle);
    esc1.write(max_thorttle);
    esc2.write(max_thorttle);
    esc3.write(max_thorttle);
    Serial.println("Max Calibrated!");
    delay(2000);
  }

  delay(2000);
  Serial.println("Calibrating Max to Min PPM");
  //calibrting zero to full
  for (i = min_thorttle; i >= center_throttle; i--) {
    esc0.write(i);
    esc1.write(i);
    esc2.write(i);
    esc3.write(i);

    Serial.println(i);
    blink();
  }
  if (i == min_thorttle) {
    esc0.write(min_thorttle);
    esc1.write(min_thorttle);
    esc2.write(min_thorttle);
    esc3.write(min_thorttle);

    Serial.println("Max Calibrated!");
    delay(2000);
  }


  delay(2000);

  Serial.println("ESC - Calibrated - Completed");
}

void esc_calibration() {
}


void gyro_signals(void) {  //Start I2C command

  //Configuring the MPU-6050 Gyroscope
  Wire.beginTransmission(0x68);
  //Switch on the low pass filter
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

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
}

void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;                                        //Calculate the P
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;  //Calculate the I
  if (Iterm > 400) Iterm = 400;                                   //Avoiding Integral Lineup
  else if (Iterm < -400) Iterm = -400;                            //Saving the number of the previous value
  float Dterm = D * (Error - PrevError) / 0.004;                  //Calculate the D

  float PIDOutput = Pterm + Iterm + Dterm;  //PID Output

  if (PIDOutput > 400) PIDOutput = 400;
  else if (PIDOutput < -400) PIDOutput = -400;

  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void reset_pid(void) {  //Reset the PID variables and Function
  PrevErrorRateRoll = 0;
  PrevErrorRatePitch = 0;
  PrevErrorRateYaw = 0;
  PrevItermRateRoll = 0;
  PrevItermRatePitch = 0;
  PrevItermRateYaw = 0;
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

  esc0.setPeriodHertz(500);  // Standard 50hz servo
  esc1.setPeriodHertz(500);  // Standard 50hz servo
  esc2.setPeriodHertz(500);  // Standard 50hz servo
  esc3.setPeriodHertz(500);  // Standard 50hz servo

  esc0.attach(esc_pin0, min_thorttle, max_thorttle);
  esc1.attach(esc_pin1, min_thorttle, max_thorttle);
  esc2.attach(esc_pin2, min_thorttle, max_thorttle);
  esc3.attach(esc_pin3, min_thorttle, max_thorttle);

  while (!Serial)
    ;
  Serial.println("Hello_ESC_Calibration!");
  delay(500);

  Serial.println("ESC_A_B_C_D");

  arming_signal();
  delay(1000);
  esc2.writeMicroseconds(center_throttle);
  delay(2000);

  LoopTimer = micros();  //Start the Timer
}

void loop() {
  gyro_signals();  //Calling Gyro Function

  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  //Calculate the Error for PID Control
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  //PID Equation
  //PID - Roll Equation
  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll = PIDReturn[0];
  PrevErrorRateRoll = PIDReturn[1];
  PrevItermRateRoll = PIDReturn[2];
  //PID - Pitch Equation
  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch = PIDReturn[0];
  PrevErrorRatePitch = PIDReturn[1];
  PrevItermRatePitch = PIDReturn[2];
  //PID - Yaw Equation
  pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
  InputYaw = PIDReturn[0];
  PrevErrorRateYaw = PIDReturn[1];
  PrevItermRateYaw = PIDReturn[2];

  if (InputThrottle > 1650) InputThrottle = 1650;  //Limit the Throttle of the motor output

//
  MotorInput1 = 1.024 * (InputThrottle - InputRoll - InputPitch - InputYaw);
  MotorInput2 = 1.024 * (InputThrottle - InputRoll + InputPitch + InputYaw);
  MotorInput3 = 1.024 * (InputThrottle + InputRoll + InputPitch - InputYaw);
  MotorInput4 = 1.024 * (InputThrottle + InputRoll - InputPitch + InputYaw);



}
