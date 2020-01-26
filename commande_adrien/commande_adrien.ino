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


void setup()
{
  // Robot instance
  // Robot r = Robot();
  // r.getWaypoints().setListWaypoints(waypointsX, waypointsY, nWaypoints);
  // r.Kp = 50.0;  // Should probably be passed to the constructor
  // r.Ki = 20.0;
  // r.Kd = 5.0;

  // Initialization of some things
  Serial.begin(115200);
  currentTime = micros();
}


void loop()
{
  previousTime = currentTime;



  // --- Wait for next iteration ---
  do {currentTime = micros();} 
  while ((currentTime - previousTime) < (sampleTime * 1000000));
}
