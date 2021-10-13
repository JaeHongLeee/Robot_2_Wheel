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
    void motor_right_A();
    void motor_right_B();
    void motor_left_A();
    void motor_left_B(); 
  
};

#endif
