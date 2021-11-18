#include <MPU9250_asukiaaa.h> //IMUsensor library

#ifndef Motor_move_h
#define Motor_move_h

#include "Arduino.h"

//Motor move class
class Motor_move {
  private:
  
  public:
    //car move method
    void forward_move();
    void back_move();

    //PWM move method
    
};

//Interrupt Class
class InterruptLibrary {
  private:
 
  public:
    //Hardware Interrupt
    void IRAM_ATTR isrPinA_R();
    void IRAM_ATTR isrPinB_R();
    void IRAM_ATTR isrPinA_L();
    void IRAM_ATTR isrPinB_L();

    //Timer Interrupt
    void interrupt_setup();    
};

//I2Cbus Class



//Move Show Class
class Move_show {
   
   public: 
    void motor_show_status();
    void speed_status();
    
};
#endif
