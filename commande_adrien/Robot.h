#pragma once

#include "Arduino.h"
#include "Waypoint.h"
#include "Motor.h"
#include "Encoder.h"
#include "Pose.h"

class Robot 
{

  private:
    // Attributes
    Motor leftMotor;
    Motor rightMotor;
    Encoder leftEncoder;
    Encoder rightEncoder;
    Pose pose;
    Pose poseEncoders;
    Pose poseCamera;
    float vR;
    float vL;
    float v;
    float omega;
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
    Waypoint waypoints;

    // Methods
    Robot();
    void updateVelocities(); // Compute wheels linear velocities
    void updatePoseEncoders(); // Compute the new pose of the robot using only encoders
    void sensorFusion(); // Performs data fusion to estimate the robot pose using sensors information, currently not implemented
    void navigate(); // Check when to change to the next waypoint
    void computePIDOutput(float sampleTime); // Compute the PID ouput
    void drive();  // Compute and sends the PWM to the motors 

    // Getters and setters
    Waypoint getWaypoints() {return this->waypoints;}
    Motor getLeftMotor() {return this->leftMotor;}
    Motor getRightMotor() {return this->rightMotor;}
    Encoder getLeftEncoder() {return this->leftEncoder;}
    Encoder getRightEncoder() {return this->rightEncoder;}
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


};
