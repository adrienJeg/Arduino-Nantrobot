#include "Arduino.h"

class Sonar
{
  private:
    float obstaceDist;

  public:
    Sonar();

    float getObstaceDist() {return this->obstaceDist;}
    void setObstaceDist(float obstaceDist) {this->obstaceDist = obstaceDist;}
};