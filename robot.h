#ifndef ROBOT_H
#define ROBOT_H
#include "motor.h"
#include "sensor.h"
#include "pid.h"

class Robot {
  private:
    // ultrasonic sensor readings
    int distanceForward = 0;
    int distanceLeft = 0;
    int distanceRight = 0;
  
    // actual velocity (ticks per 50 ms)
    double velocityLeft = 0;
    double velocityRight = 0;

    double motorPwmLeft = 0;
    double motorPwmRight = 0;

    // target velocities of motors (ticks per 50 ms)
    double targetVelocityLeft = 0;
    double targetVelocityRight = 0;

    // robot measurements
    float wheelRadius = 2.5; // inches
    float wheelAxelLength = 17; // inches

    // private step functions
    void readSensors();
    void stepNavigation();
    void stepWallFollow();
    void stepObstacleAvoidance();
    void stepBumpers();
    void stepMotors();

    void setWaypoint();
    bool isNavigationComplete();
    bool isWaypointComplete();
  
  public:
    Sensor sensorForward = Sensor(50, 51);
    Sensor sensorLeft = Sensor(53, 52);
    Sensor sensorRight = Sensor(48, 49);
    Motor motorLeft = Motor(2, 24, 25);
    Motor motorRight = Motor(3, 23, 22);

    //PID Regulators
    PID regulatorMotorLeft = PID(&velocityLeft, &motorPwmLeft, &targetVelocityLeft, 7, 8, 0, DIRECT);
    PID regulatorMotorRight = PID(&velocityRight, &motorPwmRight, &targetVelocityRight, 7, 8, 0, DIRECT);
    PID regulatorNavigationOmega = PID(&theta, &omega, &desiredTheta, 5, 0.5, 0, DIRECT);

    // velocities (ticks per 50 ms)
    float velocityFast = 20;
    float velocityModerate = 15;
    float velocitySlow = 7;

    // target position of robot
    float targetX = 0;
    float targetY = 0; 
    float targetTheta = 0;

    // current position of robot
    float x = 0;
    float y = 0;
    double theta = 0; // angle of robot (in radians)

    // error stats of current position compared to target
    double omega = 0;
    double headingError = 0; // radians
    double desiredTheta = 0; // radians
    float targetDistance = sqrt(((targetX - x)*(targetX - x)) + ((targetY - y)*(targetY - y))); // inches

    // instance functions
    void setUp();
    void step();
    void drive();
    void setVelocity(int, int);

    // wheel encoder handlers. these have to be static :(
    static void handleEncoderTickLeft();
    static void handleEncoderTickRight();

    void printStats();

    // update stats functions that are called toward end of step()
    void updateHeadingError();
    void updateOdometry();
    
    Robot();
    ~Robot();
};

#endif
