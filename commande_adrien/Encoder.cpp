/*
  Encoder.cpp - Library for getting the pulse count from encoders with a two-channel Hall effect sensor.
  Created by Adrien Jégourel, January 26, 2020.
*/
#include "Encoder.h"

Encoder *Encoder::instances[2] = {NULL, NULL};

void Encoder::encoderISR()
// Interruption Service Routine updating the pulse count
{
  value = (digitalRead(PIN_B_OUTPUT) << 1 | digitalRead(PIN_A_OUTPUT)) << 2;
  index = index | value;

  // Attention cette fois c'est un signe "-" pour que le sens de rotation soit 
  // compté positivement de la même manière que pour l'autre roue :
  pulseCount = pulseCount + updatePulseArray[index] * dirReference; 
  index = value >> 2;
}


Encoder::Encoder(int PIN_A_OUTPUT, int PIN_B_OUTPUT, signed int dirReference)
{
  this->PIN_A_OUTPUT = PIN_A_OUTPUT;
  this->PIN_B_OUTPUT = PIN_B_OUTPUT;
  this->dirReference = dirReference;

  value = 0;
  pulseCount = 0;
  index = digitalRead(PIN_B_OUTPUT) << 1 | digitalRead(PIN_A_OUTPUT);

  // Setup the interruption on the right pins
  // ISR in classes is a big deal, see : https://arduino.stackexchange.com/questions/20608/pass-classs-public-function-as-an-argument-to-external-command
  if (PIN_A_OUTPUT == 19 && PIN_B_OUTPUT == 18)
  {
    attachInterrupt(digitalPinToInterrupt(PIN_A_OUTPUT), leftISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_B_OUTPUT), leftISR, CHANGE);
    instances[0] = this;
  }
  if (PIN_A_OUTPUT == 3 && PIN_B_OUTPUT == 2)
  {
    attachInterrupt(digitalPinToInterrupt(PIN_A_OUTPUT), rightISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_B_OUTPUT), rightISR, CHANGE);
    instances[1] = this;
  }
}

