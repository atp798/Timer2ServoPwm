/*
  Timer2ServoPwm.h - Interrupt driven Servo library for Arduino using 8 bit timer2. V1.0.0
  Copyright (c) 2020, Straka.
  All rights reserved.
  MIT License.
*/
#include <Timer2ServoPwm.h>

Timer2Servo servo[4];
Timer2Pwm pwm[4];

void setup() {
  servo[0].attach(3);
  servo[1].attach(4);
  servo[2].attach(5);
  servo[3].attach(6);
  pwm[0].attach(8);
  pwm[1].attach(9);
  pwm[2].attach(10);
  pwm[3].attach(11);
}

void loop() {
  static int val = 0;
  val += 1;
  int i;
  for(i=0;i<4;++i){
    servo[i].write(val+i);
    pwm[i].write(val+i);
  }
  val %= 180;
  if(val == 0){
    delay(1000);
  }
  delay(100);
}
