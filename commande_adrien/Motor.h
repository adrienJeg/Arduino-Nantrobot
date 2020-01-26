/*
  Motor.h - Library for controlling a motor.
  Created by Adrien Jégourel, January 25, 2020.
*/
#pragma once

#include "Arduino.h"

class Motor
{
  private:
    int PIN_DIR;
    int PIN_PWM;
    int PWM;
    signed int dirReference;
    
  public:
    Motor(int PIN_DIR, int PIN_PWM, signed int dirReference); // Constructor
    void setMotor();

    // Getters and Setters
    int getPWM(){return PWM;};
    void setPWM(int newPWM){PWM = newPWM;};
};
