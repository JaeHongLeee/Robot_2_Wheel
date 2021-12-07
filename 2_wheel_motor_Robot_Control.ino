#include <MPU9250_asukiaaa.h> //IMUsensor library
#include <Wire.h> //I2C Communication
//#include <WiFi.h>
//#include <WiFiUdp.h>
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
#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif

//PWM Channel Setting
#define LEDC_TIMER_13_BIT 13        //13bit,   2^13-1 = calculate duty 8191
#define LEDC_BASE_FREQ 5000

//encoder setting
InterruptLibrary encoder_RA;
InterruptLibrary encoder_RB;
InterruptLibrary encoder_LA;
InterruptLibrary encoder_LB;

//MPU9250_asukiaaa IMUsensor;
MPU9250_asukiaaa IMUsensor;

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
  //MPU9250_asukiaaa IMUsensor;
  //IMUsensor setting(I2C 통신)
  #ifdef _ESP32_HAL_I2C_H_  //'_ESP32_HAL_I2C_H_'가 정의되어 있으면 실행
    Wire.begin(SDA_PIN, SCL_PIN);
  #else                     //'_ESP32_HAL_I2C_H_'가 정의 되어 있지 않으면 실행
    Wire.begin();
  #endif

  IMUsensor.setWire(&Wire);

  IMUsensor.beginAccel();
  IMUsensor.beginMag();
  IMUsensor.beginGyro();

  //Offset setting error value
  IMUsensor.magXOffset = 66;
  IMUsensor.magYOffset = 18;
  IMUsensor.magZOffset = 41; 
  //Serial Print Start
  Serial.begin(74880);
}

void loop() {
  //Motor Control 부분
  Move_show show1;
  Move_show motor_input;

  //motor_input.speed_status();
  //show1.motor_show_status();
 
  //IMU 센서 받아오는 여기 부분 수정 필요
  /*
  IMUsensor.accelUpdate();
  Serial.print("accelX: " + String(IMUsensor.accelX()));
  Serial.print("\t");
  Serial.print("accelY: " + String(IMUsensor.accelY()));
  Serial.print("\t");  
  Serial.print("accelZ: " + String(IMUsensor.accelZ()));
  Serial.print("\t");
  Serial.print("accelSqrt: " + String(IMUsensor.accelSqrt()));
  Serial.println("\t");
  */

  IMUsensor.gyroUpdate();
  Serial.print("gyroX: " + String(IMUsensor.gyroX()));
  Serial.print("\t");
  Serial.print("gyroY: " + String(IMUsensor.gyroY()));
  Serial.print("\t");  
  Serial.print("gyroZ: " + String(IMUsensor.gyroZ()));
  Serial.println("\t");
 
  IMUsensor.magUpdate();
  Serial.print("magX: " + String(IMUsensor.magX()));
  Serial.print("\t");
  Serial.print("magY: " + String(IMUsensor.magY()));
  Serial.print("\t");  
  Serial.print("magZ: " + String(IMUsensor.magZ()));
  Serial.print("\t");
  Serial.println("accelSqrt: " + String(IMUsensor.magHorizDirection()));
  
 
}
