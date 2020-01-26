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
    signed int dirReference;  // 1 if the motor goes forward with a positive PWM, -1 if not
    
  public:
    Motor(int PIN_DIR, int PIN_PWM, signed int dirReference); // Constructor
    void setMotor(int newPWM);

    // Getters and Setters
    int getPWM(){return PWM;};
};
