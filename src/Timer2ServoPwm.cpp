/*
  Timer2ServoPwm.cpp - Interrupt driven Servo library for Arduino using 8 bit timer2. V1.0.0
  Copyright (c) 2020, Straka.
  All rights reserved.
  MIT License.
*/
#include "Timer2ServoPwm/Timer2ServoPwm.h"
#include <avr/interrupt.h>

#define __DEBUG

typedef struct{
  uint8_t pin=0;
  uint8_t cycles=0;
  volatile uint8_t ticks=0;
  bool    activated=false;
}servo_t;

typedef struct{
   uint8_t pin=0;
   uint8_t ctn=255;
   volatile uint8_t ocr=0;
   uint8_t cur=0;
}pwm_t;

static bool inited = false;
static servo_t servos[MAX_SERVOS + 1];
static uint8_t servoCount=0;

static volatile uint8_t COMPACtn;
static volatile uint8_t curChan;

static pwm_t pwms[MAX_PWM];
static uint8_t pwmCount=0;

#ifdef __DEBUG
static uint32_t ovf_times;
static uint32_t compa_times;
#endif

static void initISR(){
   servos[MAX_SERVOS].activated = false;
   servos[MAX_SERVOS].cycles = PERIOD_REVISE_CYCLES;
   servos[MAX_SERVOS].ticks = PERIOD_REVISE_TICKS;

   COMPACtn = 0;
   curChan = 0;

#ifdef __DEBUG
   ovf_times = 0;
   compa_times = 0;
#endif

    TIMSK2 = 0;  // disable interrupts 
    TCCR2A = _BV(WGM21) | _BV(WGM20);  // fast PWM mode, top 0xFF 
    TCCR2B = _BV(CS21); // prescaler 8

    TCNT2 = 0;
    TIFR2 = _BV(TOV2) | _BV(OCF2A);
    TIMSK2 = _BV(TOIE2) | _BV(OCIE2A);         
   inited = true;
}

ISR(TIMER2_OVF_vect)
{
#ifdef __DEBUG
   ++ovf_times;
#endif
   for(uint8_t i=0;i<pwmCount;i++){
      pwms[i].cur++;
      if(pwms[i].cur <= pwms[i].ocr){
         digitalWrite(pwms[i].pin, HIGH);
      }else {
         digitalWrite(pwms[i].pin, LOW);
      }
      if(pwms[i].cur == pwms[i].ctn){
         pwms[i].cur = 0;
      }
   }
}

ISR(TIMER2_COMPA_vect){
#ifdef __DEBUG
  ++compa_times;
#endif
  ++COMPACtn;
  if(COMPACtn == 1){
    if(servos[curChan].activated){
      digitalWrite( servos[curChan].pin, HIGH);
    }
    OCR2A = servos[curChan].ticks;
  //}else if(COMPACtn <= servos[curChan].counter){
  }else if(COMPACtn == servos[curChan].cycles + 1){
     if(servos[curChan].activated){
        digitalWrite(servos[curChan].pin, LOW);
     }
  }else if(curChan == MAX_SERVOS && COMPACtn >= PERIOD_REVISE_CYCLES){
     curChan = 0;
     OCR2A = 0;
     COMPACtn = 0;
  }else if(COMPACtn > CYCLES_PER_SERVO){
     ++curChan;
     OCR2A = 0;
     COMPACtn = 0;
  }else if(COMPACtn > servos[curChan].cycles + 1){
     if(servos[curChan].activated){
        digitalWrite(servos[curChan].pin, LOW);
     }
  }
}

Timer2Servo::Timer2Servo(){
  if(servoCount>=MAX_SERVOS){
     servoChan_=INVALID_SERVO;
     return;
  }
  servoChan_=servoCount++;
}

uint8_t Timer2Servo::attach(uint8_t pin){
  if(!inited){
    initISR();
  }
   return attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t Timer2Servo::attach(uint8_t pin, uint16_t min, uint16_t max){
  if(this->servoChan_ < MAX_SERVOS){
     pinMode(pin, OUTPUT);
     servos[this->servoChan_].pin = pin;
     servos[this->servoChan_].activated = true;
     min_ = min;
     max_ = max;
  }
  return this->servoChan_;
}

void Timer2Servo::detach(){
   servos[this->servoChan_].activated = false;
}

void Timer2Servo::write(uint16_t value){
  if(value < MIN_PULSE_WIDTH){
    if(value > 180) value = 180;
    value = map(value, 0, 180, min_, max_);
  }
  this->writeMicroseconds(value);
}

void Timer2Servo::writeMicroseconds(uint16_t  value){
  if(servoChan_ >= MAX_SERVOS){
     return;
  }
  if(value < MIN_PULSE_WIDTH){
     value = MIN_PULSE_WIDTH;
  }
  if(value > MAX_PULSE_WIDTH){
     value = MAX_PULSE_WIDTH;
  }
  // TODO: call cli() sei() to ensure...
  //cli();
  servos[servoChan_].cycles = value * TICKS_PER_MICROSECOND / TICKS_PER_CYCLE;
  servos[servoChan_].ticks = (value * TICKS_PER_MICROSECOND) % TICKS_PER_CYCLE;
  //sei();
}

uint8_t Timer2Servo::read(){
   return map(readMicroseconds(), min_, max_, 0, 180);
}

uint16_t Timer2Servo::readMicroseconds(){
  if(servoChan_ >= MAX_SERVOS){
     return 0;
  }
  return (servos[servoChan_].cycles * TICKS_PER_CYCLE + servos[servoChan_].ticks) / TICKS_PER_MICROSECOND;
}

bool Timer2Servo::attached(){
  return servos[servoChan_].activated;
}

void Timer2Servo::debug(){
#ifdef __DEBUG
   for(uint8_t i=0;i<servoCount;i++){
      Serial.print("pin:");Serial.println(servos[i].pin);
      Serial.print("cycles:");Serial.println(servos[i].cycles);
      Serial.print("ticks:");Serial.println(servos[i].ticks);
      Serial.print("activated:");Serial.println(servos[i].activated);
   }
   Serial.print("ovf_times:"); Serial.println(ovf_times);
#endif
}

Timer2Pwm::Timer2Pwm(){
  if(pwmCount>=MAX_PWM){
     pwmChan_=INVALID_PWM;
     return;
  }
  pwmChan_=pwmCount++;
}

uint8_t Timer2Pwm::attach(uint8_t pin){
  if(!inited){
    initISR();
  }
  if(pwmChan_ < MAX_PWM){
    pinMode(pin, OUTPUT);
    pwms[pwmChan_].pin = pin;
    pwms[pwmChan_].ctn = 255;
  }
  return pwmChan_;
}

void Timer2Pwm::write(uint8_t pwm){
  pwms[pwmChan_].ocr = pwm;
}

uint8_t Timer2Pwm::read(){
  return pwms[pwmChan_].ocr;
}

void Timer2Pwm::debug(){
#ifdef __DEBUG
   for(uint8_t i=0;i<pwmCount;i++){
      Serial.print("pin:");Serial.println(pwms[i].pin);
      Serial.print("ctn:");Serial.println(pwms[i].ctn);
      Serial.print("ocr:");Serial.println(pwms[i].ocr);
      Serial.print("cur:");Serial.println(pwms[i].cur);
   }
   Serial.print("compa_times:"); Serial.println(compa_times);
#endif
}
