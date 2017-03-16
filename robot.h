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
    Sensor sensorRight = Sensor(6, 3);
    void setUp();
    void safeMovementCheck();
    void moveForward();
    void moveBackward();
    void turnLeft(int);
    void turnRight(int);
    void moveStop();
    Robot();
    ~Robot();
};

#endif
