//these values copied from the BlHeli configuration
#define ESC_MIN_THROTTLE 1080      //PPM Min Throttle
#define ESC_MAX_THROTTLE 1852      //PPM Max Throttle
#define ESC_REVERSE_THROTTLE 1488  //PPM Center Throttle

#define ESC_ARM_SIGNAL 1080  //ESC Arming Signal
#define ESC_ARM_TIME 2000    //ESC Arming Time (hold 2 seconds to arm the ESC) - Safety

#define ESC1_PIN 9  //ESC PWM Pin

#define ESC_DEADZONE_RANGE 100
#define ESC_FLUTTER_RANGE 10

// ESC control pin
#define POTENTIOMETER_PIN A0  // Analog input pin for potentiometer

#include <Servo.h>

#define esc_pin 9
//#define poten PA2

int value;

int map_val;
Servo esc;

void InitESC() {
  esc.writeMicroseconds(ESC_ARM_SIGNAL);
  unsigned long now = millis();
  while (millis() < now + ESC_ARM_TIME) {
    // wait for ESC to arm
  }
}

void setup() {
  Serial.begin(115200);
  esc.attach(esc_pin, 1080, 1852);  //(PWM pin, min PWM pulse, max PWM pulse), calibration start at PWM min 1000,
  InitESC();
  //pinMode(esc_pin, OUTPUT);
  //pinMode(poten, INPUT);
  //pinMode(esc_pin, OUTPUT);
  /*
  
  

  */
  esc.write(180);
  delay(1000);
  esc.write(0);
  delay(5000);
  esc.write(10);
}

void loop() {
  
  //analogWrite(esc_pin, 255);
}
