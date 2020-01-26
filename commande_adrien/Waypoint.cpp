#include "Waypoint.h"

Waypoint::Waypoint() {
  listWaypoints = DataQueue<Pose>(DEFAULT_MAX_ITEMS);
}

Waypoint::Waypoint(DataQueue<Pose> listWaypoints) {
  this->listWaypoints = listWaypoints;
}

/**
 * Initializes the queue with the array given. 
 * The queue size is set to max(DEFAULT_MAX_ITEMS, arraySize)
**/
Waypoint::Waypoint(Pose listPose[], int arraySize) {
  listWaypoints = DataQueue<Pose>(max(DEFAULT_MAX_ITEMS, arraySize));
  for ( int i = 0 ; i < arraySize ; ++i ) {
    listWaypoints.enqueue(listPose[i]);
  }
}

/**
 * Intializes the list of waypoints with two lists of x and y coordinates.
 * The queue size is set to max(DEFAULT_MAX_ITEMS, arraySize).
**/
Waypoint::Waypoint(float listX[], float listY[], int arraySize) {
  listWaypoints = DataQueue<Pose>(max(DEFAULT_MAX_ITEMS, arraySize));
  for ( int i = 0 ; i < arraySize ; ++i ) {
    listWaypoints.enqueue(Pose(listX[i], listY[i]));
  }
}

Pose Waypoint::getCurrent() {
  return listWaypoints.front();
}

/**
 * Sets the current waypoint to the next one in the list.
**/
void Waypoint::next() {
  listWaypoints.dequeue();
}

/**
 * Add a new waypoint at the end of the list.
**/
void Waypoint::add(Pose p) {
  listWaypoints.enqueue(p);
}

bool Waypoint::isEmpty() {
  return listWaypoints.isEmpty();
}
