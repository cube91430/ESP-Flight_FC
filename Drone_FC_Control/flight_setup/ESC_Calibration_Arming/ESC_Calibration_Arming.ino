/*
Copyright <2025> <Yudhistira Ranggawidjaya>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the “Software”), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, 
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial 
portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT 
LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


*/

#include <Arduino.h>
#include <Servo.h>

#define min_thorttle 1080
#define max_thorttle 1720
#define center_throttle 1488

#define arming 1720
#define arm_time 2000

#define esc_pin 13
#define poten A0
#define led_pin 13

int map_val;
int value;

Servo esc;

void setup() {
  Serial.begin(115200);
  Serial.println('Hello_ESC_01');
  esc.attach(esc_pin, 1080, 1720);

  pinMode(poten, INPUT);
  pinMode(led_pin, OUTPUT);

  arming_signal();
  delay(500);
  calibrating();
 
}

void loop() {

  /*
  */

  value = analogRead(poten);
  map_val = map(value, 0, 1023, 1080, 1720);

  esc.writeMicroseconds(map_val);
  
  /*
  if (map_val > 1488) {
    Serial.print(map_val);
    Serial.print("  ");
    Serial.println("Going_Up!");
  }

  else if (map_val < 1488) {
    Serial.print(map_val);
    Serial.print("  ");
    Serial.println("Going_Down!");
  }

  else if (map_val == center_throttle && (center_throttle - max_thorttle)) {
    Serial.print(map_val);
    Serial.print("  ");
    Serial.println("Standby!");
  } 
  
  */
  

}


void arming_signal() {

  esc.write(arming);
  unsigned long now = millis();
  while (millis() < now + arm_time) {
    // wait for ESC to arm
  }
}

void calibrating() {
  int i;  //initial calibration signal
  //calibrating full to zero
  for (i = min_thorttle; i <= max_thorttle; i++) {
    esc.write(i);
    blink();
    if (i == max_thorttle) {
      Serial.println("Max Calibrated!");
    }
  }
  Serial.println("Calibrating Min to Max PPM");
  delay(2000);

  //calibrting zero to full
  for (i = max_thorttle; i >= min_thorttle; i--) {
    esc.write(i);
    blink();
    if (i == min_thorttle) {
      Serial.println("Max Calibrated!");
    }
  }

  Serial.println("Calibrating Max to Min PPM");
  delay(2000);
  
}

void blink() {
  digitalWrite(led_pin, HIGH);
  delay(100);
  digitalWrite(led_pin, LOW);
  delay(100);

}
