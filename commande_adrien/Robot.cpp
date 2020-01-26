#include "Robot.h"

Robot::Robot() : leftMotor(8, 10, -1), rightMotor(7, 9, 1)
{
  wheelRadius = 0.04;
  L = 0.23;
  minDist = 0.02;
  leftMotor = Motor(8, 10, -1);
  rightMotor = Motor(7, 9, 1);
  // leftEncoder = Encoder();
  // rightEncoder = Encoder();
  vR = 0.0;
  vL = 0.0;
  v = 0.0;
  omega = 0.0;
  targetDistance = 0.0;
}
