#include "Motor_move.h"
#include "Arduino.h"
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
//PID Control Error Setting
volatile double R_error = 0;
volatile double R_old_error;
volatile double L_error = 0;
volatile double L_old_error;

//PID Control Value 
volatile double R_P_control, R_I_control, R_D_control;
volatile double R_PID_control;
volatile double L_P_control, L_I_control, L_D_control;
volatile double L_PID_control;

//PID Gain Values
double R_Kp = 0;
double R_Ki = 0;
double R_Kd = 0;
double L_Kp = 0;
double L_Ki = 0;
double L_Kd = 0;

//Position, Speed Setting
volatile int R_pos = 0;
volatile int L_pos = 0;
volatile int R_old_pos = 0;
volatile int L_old_pos = 0;
volatile int R_current_speed = 0;
volatile int L_current_speed = 0;
volatile int target_speed = 0;
volatile int target = 0;

//PWM Value Setting
volatile int R_PWM;
volatile int R_PWM_Input;
int R_duty_min = 2000;    //Dead Zone
int R_duty_max = 8191;    //Max PWM 

volatile int L_PWM;
volatile int L_PWM_Input;
int L_duty_min = 2000  ;   //Dead Zone
int L_duty_max = 8191;    //Max PWM

//Timer Interrupt vars
hw_timer_t * timer = NULL;

/*****************motor encoder method**************/
//Hardware Interrupt
void IRAM_ATTR InterruptLibrary::isrPinA_R() {
  if(digitalRead(Right_A) == 1) {
    if(digitalRead(Right_B) == 0) {
      R_pos = R_pos +1;
    }
    else {
      R_pos = R_pos -1;
    }
  }
  else {
    if(digitalRead(Right_B) == 1) {
      R_pos = R_pos +1;
    }
    else {
      R_pos = R_pos -1;
    }
  }
}
void IRAM_ATTR InterruptLibrary::isrPinB_R() {
  if(digitalRead(Right_B) == 1) {
    if(digitalRead(Right_A) == 1) {
      R_pos = R_pos +1;
    }
    else {
      R_pos = R_pos -1;
    }
  }
  else{
    if(digitalRead(Right_A) == 0) {
      R_pos =R_pos+1;
    }
    else {
      R_pos =R_pos -1; 
    }
  }
}

void IRAM_ATTR InterruptLibrary::isrPinA_L() {
  if(digitalRead(Left_A) == 1) {
    if(digitalRead(Left_B) == 0) {
      L_pos = L_pos +1;
    }
    else {
      L_pos = L_pos -1;
    }
  }
  else {
    if(digitalRead(Left_B) == 1) {
      L_pos = L_pos + 1;
    }
    else {
      L_pos = L_pos -1;
    }
  }
}
void IRAM_ATTR InterruptLibrary::isrPinB_L() {
  if(digitalRead(Left_B) == 1) {
    if(digitalRead(Left_A) == 1) {
      L_pos = L_pos +1;
    }
    else {
      L_pos = L_pos -1;
    }
  }
  else{
    if(digitalRead(Left_A) == 0) {
      L_pos =L_pos+1;
    }
    else {
      L_pos =L_pos -1; 
    }
  }
}

/************Timer Interrupt***************/
//Timer Interrupt method
void IRAM_ATTR onTimer() {
  R_current_speed = R_pos - R_old_pos;
  L_current_speed = L_pos - L_old_pos;
  R_old_pos = R_pos;
  L_old_pos = L_pos;


  //PID Control 
  



  //InPWM
    
}

//Timer Interrupt Setup
void InterruptLibrary::interrupt_setup(){
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, CONTROL_PERIOD, true);
  timerAlarmEnable(timer);
}

//Motor Speed Input Method
void Move_show::speed_status() {
    if (Serial.available()) {
    target = Serial.parseInt();
      if(Serial.read() == '\n') {
        target_speed = target;
      }
  }
}

//Motor Show method
void Move_show::motor_show_status() {
  Serial.print(target_speed);
  Serial.print("\t");
  Serial.print(R_pos);
  Serial.print("\t");
  Serial.println(L_pos);
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

Motor_move New;
