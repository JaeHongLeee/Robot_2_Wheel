#include "Motor_move.h"
#define CONTROL_PERIOD 50000 //50ms

const int Right_A =35;
const int Right_B = 34;
const int Left_A = 15;
const int Left_B = 4;

const int motorA1= 23;
const int motorA2= 19;
const int motorB1= 18;
const int motorB2= 5;

double target = 0;
Move_show encoder_RA;
Move_show encoder_RB;
Move_show encoder_LA;
Move_show encoder_LB;

//Timer Interrupt
volatile bool interruptCounter = false;
hw_timer_t*timer = NULL;

void IRAM_ATTR onTimer() {
  interruptCounter = true;
}

void interrupt_init(){
  timer - timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, CONTROL_PERIOD, true);
  timerAlarmEnable(timer);
}

void setup() {

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  
  pinMode(Right_A, INPUT);
  pinMode(Right_B, INPUT);
  pinMode(Left_A, INPUT);
  pinMode(Left_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(Right_A), [](){encoder_RA.isrPinA_R(); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Right_B), [](){encoder_RB.isrPinB_R(); }, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(Left_A), [](){encoder_LA.isrPinA_L(); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Left_B), [](){encoder_LB.isrPinB_L(); }, CHANGE);
  Serial.begin(115200);
}

void loop() {
  Move_show show1;

  if (Serial.available()) {
    target = Serial.parseInt();
  }
  
  //Serial.print("target: ");
  //Serial.print(target);
  //Serial.print("\t");

  show1.motor_show_status();
  
  //move1.forward_move();
  //delay(1000);

}
