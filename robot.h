#ifndef ROBOT_H
#define ROBOT_H
#include "motor.h"
#include "sensor.h"

class Robot {
  private:
    Motor motorLeft = Motor(9, 7, 8);
    Motor motorRight = Motor(5, 4, 2);

  public:
    Sensor sensorForward = Sensor(13, 12);
    Sensor sensorLeft = Sensor(11, 10);
    void setUp();
    void safeMovementCheck();
    void moveForward();
    void moveBackward();
    void moveLeft(int);
    void moveRight(int);
    void moveStop();
    Robot();
    ~Robot();
};

#endif
