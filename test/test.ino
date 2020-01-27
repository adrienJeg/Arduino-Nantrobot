#include "Arduino.h"
#include "Robot.h"
#include "Pose.h"
#include "Waypoint.h"
#include "Motor.h"
#include "Encoder.h"

Robot rob = Robot();

void setup() {
  Serial.begin(9600);
  int nWaypoints = 5;
  float waypointsX[] = {0.0, 0.0, 0.5, 0.5, 0.0}; // en m
  float waypointsY[] = {0.0, 0.5, 0.5, 0.0, 0.0}; // en m
  Waypoint w = Waypoint();
  w.setListWaypoints(waypointsX, waypointsY, nWaypoints);
  rob.getWaypoints().setListWaypoints(waypointsX, waypointsY, nWaypoints);
  w.add(Pose(3,3));
  rob.getWaypoints().add(Pose(3,3));
  Serial.print("item count rob ");
  Serial.println(rob.getWaypoints().getListWaypoints().item_count());
  Serial.print("item count w ");
  Serial.println(w.getListWaypoints().item_count());
}

void loop() {
  Serial.print("xc ");
  Serial.print(rob.getWaypoints().getCurrent().getX());
  Serial.print("\t yc ");
  Serial.println(rob.getWaypoints().getCurrent().getY());
  rob.getWaypoints().next();
  Serial.println("next()");
  Serial.print("xc ");
  Serial.print(rob.getWaypoints().getCurrent().getX());
  Serial.print("\t yc ");
  Serial.println(rob.getWaypoints().getCurrent().getY());
  delay(100000);
}
