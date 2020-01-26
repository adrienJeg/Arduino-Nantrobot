#pragma once

#include "Arduino.h"
#include "Pose.h"
#include "Vector.h"

class Waypoint
{
  private:
    Vector<Pose> listWaypoints;

  public:
    Waypoint();
    Waypoint(Vector<Pose> listWaypoints);
    Waypoint(float listX[], float listY[], int arraySize);
    void next();
    void add(Pose p);
    Pose getCurrent();

    Vector<Pose> getListWaypoints() {return this->listWaypoints;};

};
