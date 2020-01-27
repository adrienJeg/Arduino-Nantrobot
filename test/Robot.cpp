#include "Robot.h"

// Constants variables not likely to change
#define PIN_MOTOR_R_DIR 7
#define PIN_MOTOR_L_DIR 8
#define PIN_MOTOR_R_PWM 9
#define PIN_MOTOR_L_PWM 10

#define PIN_A_OUTPUT_LEFT_ENCODER 3
#define PIN_B_OUTPUT_LEFT_ENCODER 2
#define PIN_A_OUTPUT_RIGHT_ENCODER 19
#define PIN_B_OUTPUT_RIGHT_ENCODER 18

signed int leftMotorDirReference = -1, rightMotorDirReference = 1;


Robot::Robot() : leftMotor(PIN_MOTOR_L_DIR, PIN_MOTOR_L_PWM, leftMotorDirReference), 
                 rightMotor(PIN_MOTOR_R_DIR, PIN_MOTOR_R_PWM, rightMotorDirReference),
                 leftEncoder(PIN_A_OUTPUT_LEFT_ENCODER, PIN_B_OUTPUT_LEFT_ENCODER, leftMotorDirReference),
                 rightEncoder(PIN_A_OUTPUT_RIGHT_ENCODER, PIN_B_OUTPUT_RIGHT_ENCODER, rightMotorDirReference)
{
  wheelRadius = 0.04; 
  L = 0.23;
  minDist = 0.02;
  vR = 0.0;
  vL = 0.0;
  v = 1.0;
  omega = 0.0;
  angleError = 0.0;
  sumError = 0.0;
  deltaError = 0.0;
  commandDelta = 0.0;
}

void Robot::updatePoseEncoders()
/* 
 *  Compute new robot pose using encoders odometry
 */
{
  // Gives the linear distance achieved by turning the wheel of une encoder pulse.
  // Our motor (pololu 3216) has a 34:1 gearbox, and the encoder gives 48 pulses per motor shaft turn
  float distCoeff = 2.0*PI*wheelRadius / (34.0 * 48.0);  

  static signed long oldLeftPulseCount = 0, oldRightPulseCount = 0;
  
  // Compute the linear distance of each wheel achieved since last loop iteration 
  float leftDist = distCoeff*(leftEncoder.getPulseCount() - oldLeftPulseCount);
  float rightDist = distCoeff*(rightEncoder.getPulseCount() - oldRightPulseCount);

  // Save the pulse values for next iteration
  oldLeftPulseCount = leftEncoder.getPulseCount();
  oldRightPulseCount = rightEncoder.getPulseCount();
  
  // Compute delta distance and angles
  float dDist = (rightDist + leftDist) / 2.0;
  float dAngl = (rightDist - leftDist) / L;
  
  // Compute new robot pose
  poseEncoders.setX(poseEncoders.getX() + dDist*cos(poseEncoders.getTheta() + dAngl / 2.0));
  poseEncoders.setY(poseEncoders.getY() + dDist*sin(poseEncoders.getTheta() + dAngl / 2.0));
  poseEncoders.setTheta(poseEncoders.getTheta() + dAngl);
  
}


void Robot::sensorFusion()
{
  pose = Pose(poseEncoders);
}


void Robot::navigate()
{
  float targetDistance = pose.distance(waypoints.getCurrent());
  Serial.println(targetDistance);
  if (targetDistance < minDist)
  {
    waypoints.next();
  }
}

void Robot::computePIDOutput(float sampleTime) {
  float thetaDesired = atan2(waypoints.getCurrent().getY() - pose.getY(),
                        waypoints.getCurrent().getX() - pose.getX());
  thetaDesired = atan2(sin(thetaDesired), cos(thetaDesired));

  // Angle error
  float newAngleError = thetaDesired - pose.getTheta();
  float deltaError = newAngleError - angleError;
  angleError = newAngleError;

  // Integral term
  sumError = sumError + angleError * sampleTime;

  // PID output
  omega = round(Kp * angleError + Kd * deltaError + Ki * sumError);
}


void Robot::updateVelocities()
{
  vR = (2*v + omega * L) / (2.0 * wheelRadius);
  vL = (2*v - omega * L) / (2.0 * wheelRadius);
}


void Robot::drive() {
  int pwmMotorL = vL * 5.5;
  int pwmMotorR = vR * 5.5;

  // Constrain the values of the PWM
  pwmMotorL = constrain(pwmMotorL, -70, 70);
  pwmMotorR = constrain(pwmMotorR, -70, 70);

  // Send the PWM to the motors
  leftMotor.setMotor(pwmMotorL);
  rightMotor.setMotor(pwmMotorR);
}
