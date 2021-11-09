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

//PWM Setting
#define LEDC_TIMER_13_BIT 13
#define LEDC_BASE_FREQ 5000

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
double R_Kp = 6;
double R_Ki = 0;
double R_Kd = 0;

double L_Kp = 2;
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
//Right PWM
volatile int R_PWM;
volatile int R_PWM_Input;
int R_duty_min = 5600;    //Dead Zone
int R_duty_max = 8191;    //Max PWM 

//Left PWM
volatile int L_PWM;
volatile int L_PWM_Input;
int L_duty_min = 5750  ;   //Dead Zone
int L_duty_max = 8191;    //Max PWM

//Timer Interrupt vars
hw_timer_t * timer = NULL;

volatile int R_PID_Error = 1;
volatile int L_PID_Error = 1;
volatile int R_station = 0;
volatile int L_station = 0;
volatile int R_clamping = 1;
volatile int L_clamping = 1;

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
  if(target_speed<0) {
    target_speed = -target_speed;
  }
  
   R_error = target_speed - R_current_speed;
   R_P_control = R_Kp * R_error;
   R_I_control += (R_clamping*R_Ki*R_error);
   R_D_control = R_Kd*(R_error-R_old_error);
   R_old_error = R_error;
   R_PID_control = (R_P_control + R_I_control + R_D_control);
  
   L_error = target_speed - L_current_speed;
   L_P_control = L_Kp * L_error;
   L_I_control += (L_clamping*L_Ki*L_error);
   L_D_control = L_Kd*(L_error-L_old_error);
   L_old_error = L_error;
   L_PID_control = (L_P_control + L_I_control + L_D_control);
  
  //Clamping 
  if (R_PID_control * R_error >0) {
    R_PID_Error =1;
  }
  else {
    R_PID_Error = 0;
  }

  if(L_PID_control * L_error >0) {
    L_PID_Error = 1;
  }
  else {
    L_PID_Error = 0;
  }

  //anti-wideup
  if(R_PID_Error * R_station == 1) {
    R_clamping = 0;
  } else {
    R_clamping = 1;
  }

  if(L_PID_Error * L_station == 1) {
    L_clamping = 0;
  } else {
    L_clamping = 1;
  }

  //InPWM Setting
  if(target_speed >= 0){
    R_PID_control= constrain(R_PID_control, 0 , R_duty_max);
    R_PID_control=map(target_speed, 120, 290, 0, 8191);
    R_PWM=map(R_PID_control, 0, 8191, R_duty_min, R_duty_max);
    R_PWM_Input=R_PWM;

    L_PID_control= constrain(L_PID_control, 0, L_duty_max);
    L_PID_control= map(target_speed, 120, 290, 0, 8191);
    L_PWM=map(L_PID_control, 0, 8191, L_duty_min, L_duty_max);
    L_PWM_Input=L_PWM;
  }
  else if(target_speed < 0){
    R_PID_control= constrain(R_PID_control, 0, R_duty_max);
    R_PID_control= map(target_speed, 120, 290, 0, 8191);
    R_PID_control=-R_PID_control;
    R_PWM= map(R_PID_control, -1, -8191, R_duty_min, R_duty_max);
    R_PWM_Input=R_PWM;

    L_PID_control= constrain(L_PID_control, 0, L_duty_max);
    L_PID_control= map(target_speed, 120, 290, 0, 8191);
    L_PID_control=-L_PID_control;
    L_PWM= map(L_PID_control, -1, -8191, L_duty_min, L_duty_max);
    L_PWM_Input=L_PWM;
  }

  //Motor Move InPWM Input
  if(target > 0) {
    ledcWrite(0, R_PWM_Input);
    ledcWrite(1, 0);
    ledcWrite(2, L_PWM_Input);
    ledcWrite(3,0);
  }
  else if (target < 0) {
    ledcWrite(0, 0);
    ledcWrite(1, R_PWM_Input);
    ledcWrite(2, 0);
    ledcWrite(3, L_PWM_Input);    
  }
  else{
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    ledcWrite(2, 0);
    ledcWrite(3, 0);
  } 
}

void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255){
  uint32_t duty = (LEDC_BASE_FREQ / valueMax) * min(value, valueMax);
  ledcWrite(channel, duty);
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
  if(target > 0){
    Serial.print(target_speed);
    Serial.print("\t");
  }
  else if(target <0){
    Serial.print(-target_speed);
    Serial.print("\t");
  }
  else {
    Serial.print(target_speed);
    Serial.print("\t");
  }
  //Serial.print(R_pos);
  //Serial.print("\t");
  //Serial.print(L_pos);
  //Serial.print("\t");
  Serial.print("R SPEED: ");
  Serial.print(R_current_speed);
  Serial.print("\t");
  Serial.print("L_SPEED: ");
  Serial.println(L_current_speed);
}



Motor_move New;
