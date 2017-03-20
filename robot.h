#ifndef ROBOT_H
#define ROBOT_H
#include "motor.h"
#include "sensor.h"

class Robot {
  public:
    Sensor sensorForward = Sensor(50, 51);
    Sensor sensorLeft = Sensor(53, 52);
    Sensor sensorRight = Sensor(48, 49);
    Motor motorLeft = Motor(3, 24, 25);
    Motor motorRight = Motor(2, 22, 23);
    void setUp();
    void accelerate(int endSpeed, int timeDelay = 500);
    void decelerate(int endSpeed, int timeDelay = 500);
    void moveForward(int speed = 100);
    void moveBackward(int speed = 100);
    void turnLeft(int degreeOfTurn, int speed = 100);
    void turnRight(int degreeOfTurn, int speed = 100);
    void moveStop();
    static void handleEncoderTickLeft();
    static void handleEncoderTickRight();
    Robot();
    ~Robot();
};

#endif
