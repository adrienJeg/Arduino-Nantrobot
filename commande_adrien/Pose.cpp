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
  return sqrt((x - p.getX())*(x - p.getX()) + (y - p.getY())*(y - p.getY()));
}

float Pose::distance(float x, float y) {
  return sqrt((this->x - x)*(this->x - x) + (this->y - y)*(this->y - y));
}
