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
  angleError = 0.0;
  sumError = 0.0;
  deltaError = 0.0;
  commandDelta = 0.0;
}


void Robot::computePIDOutput(float sampleTime) {
  float thetaDesired = atan2(waypoints.getCurrent().getY() - pose.getY(),
                        waypoints.getCurrent().getX() - pose.getX());
  thetaDesired = atan2(sin(thetaDesired), cos(thetaDesired));

  // Angle error
  float newAngleError = thetaDesired - pose.getTheta;
  float deltaError = newAngleError - angleError;
  angleError = newAngleError;

  // Integral term
  sumError = sumError + angleError * sampleTime;

  // PID output
  omega = round(Kp * angleError + Kd * deltaError + Ki * sumError);
}