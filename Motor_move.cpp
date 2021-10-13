#include "Motor_move.h"
#include "Arduino.h"

#define A 34
#define B 35
#define C 15
#define D 4

const int motorA1= 23;
const int motorA2= 19;
const int motorB1= 18;
const int motorB2= 5;

int pos = 0;

//motor encoder method
void Move_show::motor_right_A() {
  if(digitalRead(A) == 1) {
    if(digitalRead(B) == 0) {
      pos = pos +1;
    }
    else {
      pos = pos -1;
    }
  }
  else {
    if(digitalRead(B) == 1) {
      pos = pos + 1;
    }
    else {
      pos = pos -1;
    }
  }
}
void Move_show::motor_right_B() {
  if(digitalRead(B) == 1) {
    if(digitalRead(A) == 1) {
      pos = pos +1;
    }
    else {
      pos = pos -1;
    }
  }
  else{
    if(digitalRead(A) == 0) {
      pos =pos+1;
    }
    else {
      pos =pos -1; 
    }
  }
}

void Move_show::motor_left_A() {
  if(digitalRead(C) == 1) {
    if(digitalRead(D) == 0) {
      pos = pos +1;
    }
    else {
      pos = pos -1;
    }
  }
  else {
    if(digitalRead(D) == 1) {
      pos = pos + 1;
    }
    else {
      pos = pos -1;
    }
  }
  
}
void Move_show::motor_left_B() {
  if(digitalRead(D) == 1) {
    if(digitalRead(C) == 1) {
      pos = pos +1;
    }
    else {
      pos = pos -1;
    }
  }
  else{
    if(digitalRead(C) == 0) {
      pos =pos+1;
    }
    else {
      pos =pos -1; 
    }
  }
}

//car move method
void Motor_move::back_move(){
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

void Motor_move::forward_move() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}

//motor show method 

Motor_move New;
