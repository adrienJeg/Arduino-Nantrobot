#include "Arduino.h"
#include "Robot.h"
#include "Pose.h"
#include "Waypoint.h"
#include "Motor.h"
#include "Encoder.h"

// --- Waypoints ---
int nWaypoints = 5;
float waypointsX[] = {0.0, 0.0, 0.5, 0.5, 0.0}; // en m
float waypointsY[] = {0.0, 0.5, 0.5, 0.0, 0.0}; // en m

// --- Time ---
long currentTime, previousTime;  // in microseconds
float sampleTime = 0.02; // period of the main loop, in seconds (here, frequency of 100 Hz)

// Robot instance
Robot rob = Robot();
int displayCounter;
bool displayOn = true;

void setup()
{ 
  
  rob.waypoints.setListWaypoints(waypointsX, waypointsY, nWaypoints);
  rob.Kp = 50.0;  // Should probably be passed to the constructor
  rob.Ki = 20.0;
  rob.Kd = 5.0;

  // Initialization of some things
  Serial.begin(115200);
  currentTime = micros();
}


void loop()
{
  previousTime = currentTime;
  rob.updatePoseEncoders();
  rob.sensorFusion();
  rob.navigate();
  rob.computePIDOutput(sampleTime);
  rob.updateVelocities();
  rob.drive();
  //rob.getWaypoints().next();
  if (displayCounter > 25 && displayOn)
  {
    displayCounter = 0;
    Serial.print("xc ");
    Serial.print(rob.waypoints.getCurrent().getX());
    Serial.print("\t yc ");
    Serial.println(rob.waypoints.getCurrent().getY());

  }
  displayCounter++;


  // --- Wait for next iteration ---
  do {currentTime = micros();} 
  while ((currentTime - previousTime) < (sampleTime * 1000000));
}
