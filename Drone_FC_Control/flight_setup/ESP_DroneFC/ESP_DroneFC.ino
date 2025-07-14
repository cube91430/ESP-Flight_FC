#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
//#include <Adafruit_AHTX0.h>
#include <ESP32Servo.h>

//Adafruit_AHTX0 aht;       //Temperature
Adafruit_BMP280 bmp;   //Altitude
Adafruit_MPU6050 mpu;  //Gyro - Accelerometer

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

int value;
float map_val;
float pwm;
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

void setup() {
  Serial.begin(115200);
  pinMode(calib_button, INPUT);
  //ESC - Servo Initilization
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
  /*
  
  delay(500);
  //calibrating();
  esc2.write(center_throttle);
  delay(2000);
  */
}

void loop() {

  value = analogRead(poten);

  //map_val = map(value, 0, 4095, min_thorttle, max_thorttle);
  //map_val = map(value, 0, 4095, min_thorttle, max_thorttle);
  pwm = min_thorttle + ((value / 4095.0) * (max_thorttle - min_thorttle));
  //pwm = constrain(map_val, min_thorttle, max_thorttle);

  // Smooth the ramp
  if (abs((int)pwm - last_pwm) > 5) {
    esc2.writeMicroseconds((int)pwm);
    last_pwm = (int)pwm;
    Serial.println((int)pwm);
  }

  Serial.print(value);
  Serial.print("  ");
  Serial.println(pwm);

  delay(20);
}
