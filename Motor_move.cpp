#include "Motor_move.h"
#include "Arduino.h"

volatile int R_pos = 0;
volatile int L_pos = 0;

const int Right_A =35;
const int Right_B = 34;
const int Left_A = 15;
const int Left_B = 4;

const int motorA1= 23;
const int motorA2= 19;
const int motorB1= 18;
const int motorB2= 5;

void IRAM_ATTR Move_show::isrPinA_R() {
  if(digitalRead(Right_A) == 1) {
    if(digitalRead(Right_B) == 0) {
      R_pos = R_pos -1;
    }
    else {
      R_pos = R_pos +1;
    }
  }
  else {
    if(digitalRead(Right_B) == 1) {
      R_pos = R_pos - 1;
    }
    else {
      R_pos = R_pos +1;
    }
  }
}
void IRAM_ATTR Move_show::isrPinB_R() {
  if(digitalRead(Right_B) == 1) {
    if(digitalRead(Right_A) == 1) {
      R_pos = R_pos -1;
    }
    else {
      R_pos = R_pos +1;
    }
  }
  else{
    if(digitalRead(Right_A) == 0) {
      R_pos =R_pos-1;
    }
    else {
      R_pos =R_pos +1; 
    }
  }
}

void IRAM_ATTR Move_show::isrPinA_L() {
  if(digitalRead(Left_A) == 1) {
    if(digitalRead(Left_B) == 0) {
      L_pos = L_pos -1;
    }
    else {
      L_pos = L_pos +1;
    }
  }
  else {
    if(digitalRead(Left_B) == 1) {
      L_pos = L_pos - 1;
    }
    else {
      L_pos = L_pos +1;
    }
  }
}
void IRAM_ATTR Move_show::isrPinB_L() {
  if(digitalRead(Left_B) == 1) {
    if(digitalRead(Left_A) == 1) {
      L_pos = L_pos -1;
    }
    else {
      L_pos = L_pos +1;
    }
  }
  else{
    if(digitalRead(Left_A) == 0) {
      L_pos =L_pos-1;
    }
    else {
      L_pos =L_pos +1; 
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
