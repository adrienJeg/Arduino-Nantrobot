#pragma once

#include "Arduino.h"
#include "Pose.h"
#include "Queue.h"

class Waypoint
{
  private:
    const int DEFAULT_MAX_ITEMS = 20;
    DataQueue<Pose> listWaypoints;

  public:
    Waypoint();
    Waypoint(DataQueue<Pose> listWaypoints);
    Waypoint(Pose listPose[], int arraySize);
    Waypoint(float listX[], float listY[], int arraySize);
    void next();  // Sets the current waypoint to the next one in the queue
    void add(Pose p);  // Add a new waypoint at the end of the queue
    Pose getCurrent();  // Returns the current waypoint
    bool isEmpty();  // Indicates whether the queue is empty or not

    DataQueue<Pose> getListWaypoints() {return this->listWaypoints;};

};
