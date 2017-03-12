#include "robot.h"
#include "motor.h"

Robot::Robot() {
}

Robot::~Robot() {
  this->moveStop();
}

void Robot::setUp() {
  sensorForward.setUp();
  motorLeft.setUp();
  motorRight.setUp();
}

void Robot::moveForward() {
  motorLeft.forward();
  motorRight.forward();
}

void Robot::moveBackward() {
  motorLeft.backward();
  motorRight.backward();
}

void Robot::moveLeft() {
  motorLeft.backward();
  motorRight.forward();
}

void Robot::moveRight() {
  motorLeft.forward();
  motorRight.backward();
}

void Robot::moveStop() {
  motorLeft.stop();
  motorRight.stop();
}

