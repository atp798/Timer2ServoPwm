/*
  Timer2ServoPwm.h - Interrupt driven Servo library for Arduino using 8 bit timer2. V1.0.0
  Copyright (c) 2020, Straka.
  All rights reserved.
  MIT License.
*/
#include <Timer2ServoPwm.h>

#define USE_PWM

Servo servo[7];

#ifdef USE_PWM
Timer2Pwm pwm[4];
#endif

void setup() {
  servo[0].attach(8);
  servo[1].attach(12);
  servo[2].attach(5);
  servo[3].attach(3);
  servo[4].attach(9);
  servo[5].attach(10);
  servo[6].attach(4);
#ifdef USE_PWM
  pwm[0].attach(11);
  pwm[1].attach(13);
  pwm[2].attach(6);
  pwm[3].attach(7);
#endif
}

void loop() {
  static int val = 54;
  int i;
  for(i=0;i<7;++i){
    uint16_t valw = val*10+i*5;
    servo[i].write(valw);
#ifdef USE_PWM
    if(i<4){
      pwm[i].write(val+i);
    }
#endif
  }
  val += 3;
  val %= 249;
  if(val==0){
    val=54;
  }
  delay(100);
}
