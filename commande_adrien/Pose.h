#pragma once

#include "Arduino.h"

class Pose
{
  private:
    // Attributes
    float x;
    float y;
    float theta;

  public:  
    // Methods
    Pose();
    Pose(float x, float y);
    Pose(float x, float y, float theta);
    Pose(const Pose& p);
    float distance(Pose p);
    float distance(float x, float y);

    // Getters and setters
    float getX() {return this->x;}
    void setX(float x) {this->x = x;}
    float getY() {return this->y;}
    void setY(float y) {this->y = y;}
    float getTheta() {return this->theta;}
    void setTheta(float theta) {this->theta = theta;}

};