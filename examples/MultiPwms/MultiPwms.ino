/*
  Timer2ServoPwm.h - Interrupt driven Servo library for Arduino using 8 bit timer2. V1.0.0
  Copyright (c) 2020, Straka.
  All rights reserved.
  MIT License.
*/
#include <Timer2ServoPwm.h>

Timer2Pwm pwm[8];

void setup() {
  pwm[0].attach(3);
  pwm[1].attach(4);
  pwm[2].attach(5);
  pwm[3].attach(6);
  pwm[4].attach(7);
  pwm[5].attach(8);
  pwm[6].attach(9);
  pwm[7].attach(10);
}

void loop() {
  static int val = 0;
  val += 1;
  int i;
  for(i=0;i<8;++i){
    pwm[i].write(val+i);
  }
  val %= 256;
  delay(100);
}
