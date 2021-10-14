#include "Motor_move.h"

volatile int interruptCounter;

const int Right_A =35;
const int Right_B = 34;
const int Left_A = 15;
const int Left_B = 4;

const int motorA1= 23;
const int motorA2= 19;
const int motorB1= 18;
const int motorB2= 5;
int target = 0;


void setup() {

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  
  pinMode(Right_A, INPUT);
  pinMode(Right_B, INPUT);
  pinMode(Left_A, INPUT);
  pinMode(Left_B, INPUT);

//  attachInterrupt(digitalPinToInterrupt(Right_A), isrPinA_R, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(Right_B), isrPinB_R, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(Left_A), isrPinA_L, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(Left_B), isrPinB_L, CHANGE);
  Serial.begin(9600);
}

void loop() {
  Motor_move move1;
  Motor_move move2;

  if (Serial.available()) {
    target = Serial.parseInt();
  }
  
  Serial.print("target: ");
  Serial.print(target);
  Serial.print("\t");

  
  move1.forward_move();
  delay(1000);

  move2.back_move();
  delay(1000);


}
