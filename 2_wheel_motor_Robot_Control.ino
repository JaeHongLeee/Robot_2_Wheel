#include "Motor_move.h"
#define LED 2

const int motorA1= 23;
const int motorA2= 19;
const int motorB1= 18;
const int motorB2= 5;

void setup() {
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
}

void loop() {
  Motor_move move1;
  Motor_move move2;
  
  move1.forward_move();
  delay(1000);
  move2.back_move();
  delay(1000);

}
