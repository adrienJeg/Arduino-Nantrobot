#pragma once

#include "Arduino.h"
#include "Pose.h"
#include <Vector.h>

class Waypoint
{
  private:
    Pose currentPose;
    int currentIndex;
    Vector<Pose> listWaypoints;

  public:
    Waypoint();
    Waypoint(Vector<Pose> listWaypoints);
    // Waypoint(float[] listX, float[] listY);
    void next();
    void add(Pose p);

    Pose getCurrentPose() {return this->currentPose;};
    Vector<Pose> getListWaypoints() {return this->listWaypoints;};

};
