#include "Waypoint.h"

Waypoint::Waypoint() {
}

Waypoint::Waypoint(Vector<Pose> listWaypoints) {
  this->listWaypoints = listWaypoints;
}

Waypoint::Waypoint(float* listX, float* listY) {
  
}

Pose Waypoint::getCurrent() {
  return listWaypoints[0];
}

/**
 * Sets the current waypoint to the next one in the list.
**/
void Waypoint::next() {
  listWaypoints.remove(0);
}

/**
 * Add a new waypoint at the end of the list.
**/
void Waypoint::add(Pose p) {
  listWaypoints.push_back(p);
}
