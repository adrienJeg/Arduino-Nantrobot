#include "Arduino.h"
#include "Robot.h"
#include "Pose.h"
#include "Waypoint.h"
#include "Motor.h"
#include "Encoder.h"

int nWaypoints = 5;
float waypointsX[] = {0.0, 0.0, 0.5, 0.5, 0.0}; // en m
float waypointsY[] = {0.0, 0.5, 0.5, 0.0, 0.0}; // en m


void setup()
{
  Robot r = Robot();
  r.getWaypoints().setListWaypoints(waypointsX, waypointsY, nWaypoints);

}

void loop()
{
  
}
