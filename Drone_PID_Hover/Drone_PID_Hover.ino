/* --- THIS IS THE CODE TO TEST THE CALIBRATION FOR ESC's ---- */

#include "Wire.h"
#include "ESP32Servo.h"

//potentiometer - value
#define poten 36
float poten_rate;
float pwm_rate;

#define min_thorttle 1148     //Min PPM Throttle
#define max_thorttle 1832     //Max PPM Throttle
#define center_throttle 1488  //Center PPM Throttle

#define arming max_thorttle    //Arming PPM Signal
#define arm_time 2000  //ESC Arming Time

//Left ESC pin - A
#define esc_pin1 27  //Cw
#define esc_pin2 25  //CCW

//Right ESC pin - B
#define esc_pin3 4  //CW
#define esc_pin4 12  //CCW

//ESC Attach
Servo esc0;
Servo esc1;
Servo esc2;
Servo esc3;

//Pitch PID
#define Kp_Pitch 0
#define Ki_Pitch 0
#define Kd_Pitch 0

//Yaw PID
#define Kp_Yaw 0
#define Ki_Yaw 0
#define Kd_Yaw 0

//Roll PID
#define Kp_Roll 0
#define Ki_Roll 0
#define Kd_Roll 0

double dt, last_time;
double rate, desired_rate, actual_rate;

void arming_signal() {
  Serial.println("ARMING ESC!");

  esc0.write(arming);
  esc1.write(arming);
  esc2.write(arming);
  esc3.write(arming);
  unsigned long now = millis();
  while (millis() < now + arm_time) {
    // wait for ESC to arm
  }
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
    //blink();
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
    //blink();
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

void pid_equation() {

}


void setup() {

  last_time = 0;

  Serial.begin(115200);

  esc0.setPeriodHertz(480);  // Standard 50hz servo
  esc1.setPeriodHertz(480);  // Standard 50hz servo
  esc2.setPeriodHertz(480);  // Standard 50hz servo
  esc3.setPeriodHertz(480);  // Standard 50hz servo

  esc0.attach(esc_pin1, min_thorttle, max_thorttle);
  esc1.attach(esc_pin2, min_thorttle, max_thorttle);
  esc2.attach(esc_pin3, min_thorttle, max_thorttle);
  esc3.attach(esc_pin4, min_thorttle, max_thorttle);
  
  arming_signal();
  //calibrating();

}

void loop() {
  //Serial.println("Running Pin_1");
  //esc1.writeMicroseconds(1700);
  poten_rate = analogRead(poten);
  pwm_rate = map(poten_rate, 0, 4095, min_thorttle, max_thorttle);

  esc1.writeMicroseconds(pwm_rate);
  esc2.writeMicroseconds(pwm_rate);
  esc3.writeMicroseconds(pwm_rate);
  esc4.writeMicroseconds(pwm_rate);

  /*
  double now = millis();
  dt = (now - last_time); //in ms
  last_time = now;

  rate = actual_rate - desired_rate;
  */
  

}
