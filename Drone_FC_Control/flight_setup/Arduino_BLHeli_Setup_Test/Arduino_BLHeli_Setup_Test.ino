#include "Servo.h"
#include "BluetoothSerial.h"

#define min_thorttle 1080
#define max_thorttle 1720
#define center_throttle 1488

#define arming 1080
#define arm_time 2000


#define pwm0 3
#define pwm1 5
#define pwm2 9
#define pwm3 10

#define poten A5

Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

float val;
float map_val;

void arming_signal() {

  esc1.writeMicroseconds(arming);
  esc2.writeMicroseconds(arming);
  esc3.writeMicroseconds(arming);
  esc4.writeMicroseconds(arming);
  unsigned long now = millis();
  while (millis() < now + arm_time) {
    // wait for ESC to arm
  }
  
  esc1.writeMicroseconds(arming);
  esc2.writeMicroseconds(arming);
  esc3.writeMicroseconds(arming);
  esc4.writeMicroseconds(arming);
}

void setup() {
  Serial.begin(115200);
  pinMode(poten, INPUT);

  esc1.attach(pwm0, min_thorttle, max_thorttle);
  esc2.attach(pwm1, min_thorttle, max_thorttle);
  esc3.attach(pwm2, min_thorttle, max_thorttle);
  esc4.attach(pwm3, min_thorttle, max_thorttle);

  Serial.println("Firmware: EscPWMTesting_Potentiometer.ino");

  arming_signal();
  //delay(2000);
  //calibrating();
  delay(500);

  Serial.println("ESC Armed and Ready.");
}

void loop() {
  val = analogRead(poten);
  map_val = map(val, 0, 1023, min_thorttle, max_thorttle);

  Serial.print(val);

  Serial.print("  ");
  Serial.print(map_val);

  esc1.write(map_val);
  esc2.write(map_val);
  esc3.write(map_val);
  esc4.write(map_val);

  Serial.println();
}


void calibrating() {
  int i;  //initial calibration signal

  //calibrting zero to full
  for (i = max_thorttle; i >= min_thorttle; i--) {
    esc1.write(i);
    esc2.write(i);
    esc3.write(i);
    esc4.write(i);
    Serial.println(i);

    if (i == min_thorttle) {
      Serial.println("Min Calibrated!");
    }
  }
  Serial.println("Calibrating Max to Min PPM");
  //delay(2000);
}
