#include "Motor_move.h"

#define A 34
#define B 35
#define C 15
#define D 4

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
  pinMode(A, INPUT);
  pinMode(B, INPUT);
  pinMode(C, INPUT);
  pinMode(D, INPUT);
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
