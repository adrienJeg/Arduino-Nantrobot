#include "Pose.h"

Pose::Pose(){
  this->x = 0;
  this->y = 0;
  this->theta = 0;
}

Pose::Pose(float x, float y) {
  this->x = x;
  this->y = y;
  this->theta = 0;
}

Pose::Pose(float x, float y, float theta) {
  this->x = x;
  this->y = y;
  this->theta = theta;
}

Pose::Pose(const Pose& p) {
  this->x = p.x;
  this->y = p.y;
  this->theta = p.theta;
}

float Pose::distance(Pose p) {
  return sqrt((this->getX() - p.getX())*(this->getX() - p.getX()) + 
          (this->getY() - p.getY())*(this->getY() - p.getY()));
}

float Pose::distance(float x, float y) {
  return sqrt((this->getX() - x)*(this->getX() - x) + 
          (this->getY() - y)*(this->getY() - y));
}
