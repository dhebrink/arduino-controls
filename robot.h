#ifndef ROBOT_H
#define ROBOT_H
#include "motor.h"
#include "sensor.h"

class Robot {
  private:
    // actual velocity
    int velocityLeft = 0;
    int velocityRight = 0;

    // target velocities of motors
    int targetVelocityLeft = 0;
    int targetVelocityRight = 0;

    // previous encoder tick count
    int lastEncoderTickCountLeft = 0;
    int lastEncoderTickCountRight = 0;

    // robot measurements
    float wheelRadius = 2.5; // inches
    float wheelAxelLength = 17; // inches

    // private step functions
    void stepNavigation();
    void stepObstacleAvoidance();
    void stepBumpers();
    void stepMotors();

    void printStats();

    // update stats functions that are called toward end of step()
    void updateHeadingError();
    void updateOdometry();
  
  public:
    Sensor sensorForward = Sensor(50, 51);
    Sensor sensorLeft = Sensor(53, 52);
    Sensor sensorRight = Sensor(48, 49);
    Motor motorLeft = Motor(2, 24, 25);
    Motor motorRight = Motor(3, 23, 22);

    // target position of robot
    float targetX = -24;
    float targetY = 24;
    float targetTheta = 0;

    // current position of robot
    float x = 0;
    float y = 0;
    float theta = 0; // angle of robot (in radians)

    // error stats of current position compared to target
    float headingError = 0; // radians
    float targetBearing = 0; // radians
    float targetDistance = 0; // inches

    // instance functions
    void setUp();
    void step();
    void drive();
    void setVelocity(int, int);

    // wheel encoder handlers. these have to be static :(
    static void handleEncoderTickLeft();
    static void handleEncoderTickRight();
    
    Robot();
    ~Robot();
};

#endif
