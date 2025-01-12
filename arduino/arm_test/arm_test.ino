#define DIR_WRIST1 30
#define PWM_WRIST1 13
#define ENCA_WRIST1 37
#define ENCB_WRIST1 35

#define DIR_WRIST2 32
#define PWM_WRIST2 12
#define ENCA_WRIST2 39
#define ENCB_WRIST2 41

#define DIR_GRIPPY 27
#define PWM_GRIPPY 7
#define ENCA_GRIPPY 45
#define ENCB_GRIPPY 4

#define DIR_LA1 56
#define PWM_LA1 4
#define POTEN_LA1 A0

#define DIR_LA2 28
#define PWM_LA2 7
#define POTEN_LA2 A1

#include "cytrons.h"

volatile int posi1 = 0;
volatile int posi2 = 0;
volatile int posi3 = 0;



MDD10A wrist1(PWM_WRIST1, DIR_WRIST1, ENCA_WRIST1, ENCB_WRIST1);
MDD10A wrist2(PWM_WRIST2, DIR_WRIST2, ENCA_WRIST2, ENCB_WRIST2);
MDD10A grippy(PWM_GRIPPY, DIR_GRIPPY, ENCA_GRIPPY, ENCB_GRIPPY);

MDD10A motor3(PWM_LA1, DIR_LA1, 0, 0);
MDD10A motor4(PWM_LA2, DIR_LA2, 0, 0);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(ENCA_WRIST1),readEncoder1,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_WRIST2),readEncoder2,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_GRIPPY),readEncoder3,RISING);

}

void loop() {
  // put your main code here, to run repeatedly:
  int pwr = 50;


  // motor1.run(pwr); motor2.run(-pwr);
  // Serial.println(motor1.getEncoderCount());

  // delay(1);

  //wrist.run(pwr);
  //.run(pwr);
/**
  delay(1000);

  Serial.print("LA1 Position: ");
  Serial.println(digitalRead(POTEN_LA1));

  Serial.print("LA2 Position: ");
  Serial.println(digitalRead(POTEN_LA2));

  motor3.run(-pwr);
  motor4.run(-pwr);

  delay(1000);

  Serial.print("LA1 Position: ");
  Serial.println(analogRead(POTEN_LA1));

  Serial.print("LA2 Position: ");
  Serial.println(analogRead(POTEN_LA2));*/
  /**
  wrist1.run(pwr);
  wrist2.run(-pwr);
  delay(500);
  wrist1.stop();
  wrist2.stop();
  delay(500);
  wrist1.run(pwr);
  wrist2.run(-pwr);
  delay(500);

  grippy.run(pwr);
  delay(500);
  grippy.stop();
  delay(500);
  grippy.run(pwr);
  delay(500);*/

  Serial.print("LA1 Position: ");
  Serial.println(analogRead(POTEN_LA1));

  Serial.print("LA2 Position: ");
  Serial.println(analogRead(POTEN_LA2));

  Serial.print("WRIST1 ");
  Serial.println(posi1);
  Serial.print("WRIST2 :");
  Serial.println(posi2);
  Serial.print("GRIPPY :");
  Serial.println(posi3);
  delay(500);
}



void readEncoder1(){
  int b = digitalRead(ENCB_WRIST1);
  if(b > 0){
    posi1++;
  }
  else{
    posi1--;
  }
}

void readEncoder2(){
  int b = digitalRead(ENCB_WRIST2);
  if(b > 0){
    posi2++;
  }
  else{
    posi2--;
  }
}

void readEncoder3(){
  int b = digitalRead(ENCB_GRIPPY);
  if(b > 0){
    posi3++;
  }
  else{
    posi3--;
  }
}
