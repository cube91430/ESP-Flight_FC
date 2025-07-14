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
#include <ESP32Servo.h>

#define min_thorttle 1000     //1080
#define max_thorttle 2000     //1720
#define center_throttle 1500  //1488

#define arming 2000    //1720
#define arm_time 1000  //2000

#define esc_pin 27
#define poten 39

int map_val;
int value;

Servo esc;

void setup() {
  Serial.begin(115200);
  Serial.println('Hello_ESC_01');
  esc.attach(esc_pin, min_thorttle, max_thorttle);

  pinMode(poten, INPUT);
  //pinMode(led_pin, OUTPUT);

  arming_signal();
  delay(500);
  //calibrating();
}

void loop() {

  value = analogRead(poten);

  //analog poten limit to 450

  map_val = map(value, 0, 4095, min_thorttle, max_thorttle);

  esc.writeMicroseconds(map_val);

  Serial.println(value);
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
    //blink();
    if (i == max_thorttle) {
      Serial.println("Max Calibrated!");
    }
  }
  Serial.println("Calibrating Min to Max PPM");
  delay(2000);

  //calibrting zero to full
  for (i = max_thorttle; i >= min_thorttle; i--) {
    esc.write(i);
    //blink();
    if (i == min_thorttle) {
      Serial.println("Max Calibrated!");
    }
  }

  Serial.println("Calibrating Max to Min PPM");
  delay(2000);
}
