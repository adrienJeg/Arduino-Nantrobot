#include "Waypoint.h"

Waypoint::Waypoint() {
}

Waypoint::Waypoint(Vector<Pose> listWaypoints) {
  this->listWaypoints = listWaypoints;
}

/**
 * Intializes the list of waypoints with two lists of x and y coordinates.
**/
Waypoint::Waypoint(float listX[], float listY[], int arraySize) {
  for ( int i = 0 ; i < arraySize ; ++i ) {
    listWaypoints.push_back(Pose(listX[i], listY[i]));
  }
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
