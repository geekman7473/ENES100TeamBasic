#ifndef DFRTank_H_
#define DFRTank_H_

#include <Arduino.h>

//Standard PWM DC control
#define DFRTANK_E1 5   //M1 Speed Control
#define DFRTANK_E2 6     //M2 Speed Control
#define DFRTANK_M1 4    //M1 Direction Control
#define DFRTANK_M2 7    //M1 Direction Control

class DFRTank {
  private:
    int _right_motor_pwm, _left_motor_pwm;
  public:
    DFRTank();
    void init();
    void turnOffMotors();
    void setRightMotorPWM(int pwm);
    void setLeftMotorPWM(int pwm);
    int getLeftMotorPWM(){
      return _left_motor_pwm;
    };
    int getRightMotorPWM(){
      return _right_motor_pwm;
    };
};

#endif
