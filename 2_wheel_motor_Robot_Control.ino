#include <MPU9250_asukiaaa.h> //IMUsensor library
#include "Motor_move.h"

#define CONTROL_PERIOD 50000 //50ms

//Pin number setting
#define Right_A 35
#define Right_B 34
#define Left_A 15
#define Left_B 4

#define motorA1 23
#define motorA2 19
#define motorB1 18
#define motorB2 5

//IMU PIN number setting
#define SDA_PIN 21
#define SCL_PIN 22

//PWM Channel Setting
#define LEDC_TIMER_13_BIT 13        //13bit,   2^13-1 = calculate duty 8191
#define LEDC_BASE_FREQ 5000

//encoder setting
InterruptLibrary encoder_RA;
InterruptLibrary encoder_RB;
InterruptLibrary encoder_LA;
InterruptLibrary encoder_LB;

//IMUsensor setting
MPU9250_asukiaaa mySensor;


//Setup method
void setup() {
  InterruptLibrary timer_1;       //Timer Interrupt

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  
  pinMode(Right_A, INPUT);
  pinMode(Right_B, INPUT);
  pinMode(Left_A, INPUT);
  pinMode(Left_B, INPUT);

  //Hardware Interrupt Setting
  attachInterrupt(digitalPinToInterrupt(Right_A), [](){encoder_RA.isrPinA_R(); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Right_B), [](){encoder_RB.isrPinB_R(); }, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(Left_A), [](){encoder_LA.isrPinA_L(); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Left_B), [](){encoder_LB.isrPinB_L(); }, CHANGE);

  //PWM pin Setting
  ledcSetup(0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  ledcAttachPin(motorB2, 0);
  ledcSetup(1, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  ledcAttachPin(motorB1, 1);
  ledcSetup(2, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  ledcAttachPin(motorA2, 2);
  ledcSetup(3, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  ledcAttachPin(motorA1, 3);
  
  timer_1.interrupt_setup();      //Timer Interrupt
  
  Serial.begin(74880);
}

void loop() {
  Move_show show1;
  Move_show motor_input;

  motor_input.speed_status();
  show1.motor_show_status();
  
 
}
