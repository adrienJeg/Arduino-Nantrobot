#include "Arduino.h"

class Sonar
{
  private:
    float obstacleDist;

  public:
    Sonar();

    float getObstacleDist() {return this->obstacleDist;}
    void setObstacleDist(float obstacleDist) {this->obstacleDist = obstacleDist;}
};
