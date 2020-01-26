/*
  Encoder.cpp - Library for getting the pulse count from encoders with a two-channel Hall effect sensor.
  Created by Adrien Jégourel, January 26, 2020.
*/
#pragma once

#include "Arduino.h"

class Encoder
{
  private:
    // Attributes
    int PIN_A_OUTPUT;
    int PIN_B_OUTPUT;
    signed int dirReference;
    volatile int index;
    volatile int value;
    volatile signed long pulseCount;
    volatile signed long updatePulseArray[16] = {0, -1, +1, 0, +1, 0, 0, -1, -1, 0, 0, +1, 0, +1, -1, 0};

    static Encoder *instances[2];

    // Methods
    void encoderISR();

    static void leftISR()
    {
        if(Encoder::instances[0] != NULL)
            Encoder::instances[0]->encoderISR();
    }

    static void rightISR()
    {
        if (Encoder::instances[1] != NULL)
            Encoder::instances[1]->encoderISR();
    }
 
    
  public:
    // Constructor
    Encoder(int PIN_A_OUTPUT, int PIN_B_OUTPUT, signed int dirReference); 

    // Getters and Setters
    volatile signed long getPulseCount(){return pulseCount;};
};

