#include "robot.h"
#include "motor.h"

Robot::Robot() {
}

Robot::~Robot() {
  this->moveStop();
}

void Robot::setUp() {
  sensorForward.setUp();
  sensorRight.setUp();
  sensorLeft.setUp();
  motorLeft.setUp();
  motorRight.setUp();
}

void Robot::safeMovementCheck() {
  int frontDistance = this->sensorForward.getDistance();
  if (frontDistance > 0 && frontDistance <= forwardDistanceThreshold) {
    this->moveStop();
  }
}
void Robot::moveForward() {
  // this->safeMovementCheck();
  motorLeft.forward();
  motorRight.forward();
}

void Robot::moveBackward() {
  motorLeft.backward();
  motorRight.backward();
}

void Robot::turnLeft(int degreeOfTurn) {
  motorLeft.backward();
  motorRight.forward();
  // for time-delayed turning, 180 deg is roughly 1000 milliseconds
  int waitDuration = int((degreeOfTurn / 180.0) * 1000);
  Serial.println("Waiting for " + String(waitDuration));
  delay(waitDuration);
  this->moveStop();
}

void Robot::turnRight(int degreeOfTurn) {
  motorLeft.forward();
  motorRight.backward();
  int waitDuration = int((degreeOfTurn / 180.0) * 1000);
  Serial.println("Waiting for " + String(waitDuration));
  delay(waitDuration);
  this->moveStop();
}

void Robot::moveStop() {
  motorLeft.stop();
  motorRight.stop();
}

