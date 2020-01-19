/*
  Timer2ServoPwm.h - Interrupt driven Servo library for Arduino using 8 bit timer2. V1.0.0
  Copyright (c) 2020, Straka.
  All rights reserved.
  MIT License.
*/
#include <Timer2ServoPwm.h>

Timer2Servo servo;

void setup() {
  servo.attach(10);
}

void loop() {
  static int val = 0;
  val += 1;
  val %= 180;
  servo.write(val);
  if(val == 0){
    delay(1000);
  }
  delay(100);
}
