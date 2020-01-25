/*
  Motor.cpp - Library for controlling a motor.
  Created by Adrien Jégourel, January 25, 2020.
*/
#pragma once

#include "Motor.h"

Motor::Motor(const int PIN_DIR, const int PIN_PWM, signed int dirReference)
{
  // Attributes initialization
  this->PIN_DIR = PIN_DIR;
  this->PIN_PWM = PIN_PWM;
  this->dirReference = dirReference;
  PWM = 0;

  // Arduino pins initialization
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);

  if (dirReference == -1)
    digitalWrite(PIN_DIR, LOW);
  if (dirReference == 1)
    digitalWrite(PIN_DIR, HIGH);
}

void Motor::setMotor()
{
  PWM = constrain(PWM, -250, 250); // to make sure we don't overflow
  PWM = PWM * dirReference; // Different between left and right motor
  if (PWM > 0) 
  {
    digitalWrite(PIN_DIR, HIGH); // Set the rotation direction 
  }
  if (PWM <= 0) 
  {
    digitalWrite(PIN_DIR, LOW); // Set the rotation direction 
  }
  analogWrite(PIN_PWM, abs(PWM));   // Send command to motor driver pwm
}

