#pragma once

#include "Arduino.h"
#include "Waypoint.h"
#include "Motor.h"
// #include "Encoder.h"
#include "Pose.h"

class Robot 
{

  private:
    // Attributes
    Waypoint waypoints;
    Motor leftMotor;
    Motor rightMotor;
    // Encoder leftEncoder;
    // Encoder rightEncoder;
    Pose pose;
    Pose poseEncoders;
    Pose poseCamera;
    float vR;
    float vL;
    float v;
    float omega;
    float targetDistance;
    float angleError;
    float sumError;
    float deltaError;
    float commandDelta;

  public:
    // Attributes
    float wheelRadius;  // in meters
    float L;  // distance between the two wheels in meters
    float minDist;  // min distance (in meters) to consider that the robot reached a waypoint
    float Kp;  // P coefficient for the PID
    float Ki;  // I coefficient for the PID
    float Kd;  // D coefficient for the PID

    // Methods
    Robot();
    void computePIDOutput(float sampleTime);

    // Getters and setters
    Waypoint getWaypoints() {return this->waypoints;}
    Motor getLeftMotor() {return this->leftMotor;}
    Motor getRightMotor() {return this->rightMotor;}
    // Encoder getLeftEncoder() {return this->leftEncoder;}
    // Encoder getRightEncoder() {return this->rightEncoder;}
    Pose getPose() {return this->pose;}
    Pose getPoseEncoders() {return this->poseEncoders;}
    Pose getPoseCamera() {return this->poseCamera;}
    
    float getVR() {return this->vR;}
    void setVR(float vR) {this->vR = vR;}
    
    float getVL() {return this->vL;}
    void setVL(float vL) {this->vL = vL;}
    
    float getV() {return this->v;}
    void setV(float v) {this->v = v;}
    
    float getOmega() {return this->omega;}
    void setOmega(float omega) {this->omega = omega;}
    
    float getTargetDistance() {return this->targetDistance;}
    void setTargetDistance(float targetDistance) {this->targetDistance = targetDistance;}

};
