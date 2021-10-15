#ifndef Motor_move_h
#define Motor_move_h
#include "Arduino.h"

//Motor move class
class Motor_move {
  private:
  
  public:
    //car move methodnm
    void forward_move();
    void back_move();
};

//Move show class
class Move_show {
  private:

  public:
    //Hardware Interrup
    void IRAM_ATTR isrPinA_R();
    void IRAM_ATTR isrPinB_R();
    void IRAM_ATTR isrPinA_L();
    void IRAM_ATTR isrPinB_L();
    void motor_show_status(); 
};

/*
//Interrupt class
class Interrupt {
  public:
    void Hardware(int pin1, int pin2, int pin3, int pin4); 
};
*/
#endif
