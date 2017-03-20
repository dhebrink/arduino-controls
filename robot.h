#ifndef ROBOT_H
#define ROBOT_H
#include "motor.h"
#include "sensor.h"

class Robot {
  private:
    long long lastEncoderTickCountLeft = 0;
    long long lastEncoderTickCountRight = 0;
  
  public:
    Sensor sensorForward = Sensor(50, 51);
    Sensor sensorLeft = Sensor(53, 52);
    Sensor sensorRight = Sensor(48, 49);
    Motor motorLeft = Motor(2, 24, 25);
    Motor motorRight = Motor(3, 23, 22);

    float x = 0;
    float y = 0;
    float z = 0;
    float phi = 0; // angle of robot (in radians)
    
    void setUp();
    void updateOdometry();
    void stepAutonomously();
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
