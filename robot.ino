#include "robot.h"
#include "motor.h"


Robot::Robot() {
}

Robot::~Robot() {
  this->moveStop();
}

void Robot::setUp() {
  attachInterrupt(digitalPinToInterrupt(18), handleEncoderTickLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(19), handleEncoderTickRight, RISING);
  
  sensorForward.setUp();
  sensorRight.setUp();
  sensorLeft.setUp();
  motorLeft.setUp();
  motorRight.setUp();
}

void Robot::accelerate(int endSpeed, int timeDelay) {
  Serial.println("Accelerate To: " + String(endSpeed));

  // Get max of both motor speeds
  int leftSpeed = this->motorLeft.motorSpeed;
  int rightSpeed = this->motorRight.motorSpeed;
  Serial.println("Left Motor Speed: " + String(leftSpeed));
  Serial.println("Right Motor Speed: " + String(rightSpeed));

  // Set overall speed to the max of the two initially
  int currentSpeed = max(leftSpeed, rightSpeed);
  Serial.println("Matching speeds to MAX of right and left: " + String(currentSpeed));

  this->moveForward(currentSpeed);
  // Delay a bit here, so that speeds can be adjusted to match before acceleration
  delay(timeDelay);

  // Initially going to assume that speed changes are given in multiples of 10
  // TODO: Revisit this and find a better way to increment currentSpeed...?
  // Until then, this should work for current intentions.

  while (currentSpeed < endSpeed) {
    currentSpeed += 10;
    Serial.println("Speed increased to " + String(currentSpeed));
    this->moveForward(currentSpeed);
    delay(timeDelay);
  }
  Serial.println("Ending current speed, end of acceleration: " + String(currentSpeed));
}

void Robot::decelerate(int endSpeed, int timeDelay) {
  Serial.println("Decelerating To: " + String(endSpeed));

  // Get minimum of current motor speeds
  int leftSpeed = this->motorLeft.motorSpeed;
  int rightSpeed = this->motorRight.motorSpeed;
  Serial.println("Left Motor Speed: " + String(leftSpeed));
  Serial.println("Right Motor Speed: " + String(rightSpeed));

  // Set overall speed to the minimum of the two initially
  int currentSpeed = min(leftSpeed, rightSpeed);
  Serial.println("Matching speeds to MIN of right and left: " + String(currentSpeed));

  this->moveForward(currentSpeed);
  // Delay a bit here, so that speeds can be adjusted to match before deceleration
  delay(timeDelay);

  // Similar to accelerate(), going to assume values are given in increments of 10
  // TODO: Reivist this if we update the acceleration logic.
  // May also come back and find a way to not duplicate logic betwee this method and accelerate().
  while (currentSpeed > endSpeed) {
    currentSpeed -= 10;
    Serial.println("Speed decreased to " + String(currentSpeed));
    this->moveForward(currentSpeed);
    delay(timeDelay);
  }
  Serial.println("Ending speed, end of deceleration: " + String(currentSpeed));
}

void Robot::moveForward(int speed) {
  this->motorLeft.forward(speed);
  this->motorRight.forward(speed);
}

void Robot::moveBackward(int speed) {
  motorLeft.backward(speed);
  motorRight.backward(speed);
}

void Robot::turnLeft(int degreeOfTurn, int speed) {
  motorLeft.backward(speed);
  motorRight.forward(speed);
  // for time-delayed turning, 180 deg is roughly 1000 milliseconds
  int waitDuration = int((degreeOfTurn / 180.0) * 1000);
  Serial.println("Turning LEFT for " + String(waitDuration));
  delay(waitDuration);
  this->moveStop();
}

void Robot::turnRight(int degreeOfTurn, int speed) {
  motorLeft.forward();
  motorRight.backward();
  int waitDuration = int((degreeOfTurn / 180.0) * 1000);
  Serial.println("Turning RIGHT for " + String(waitDuration));
  delay(waitDuration);
  this->moveStop();
}

void Robot::moveStop() {
  motorLeft.stop();
  motorRight.stop();
}

static void Robot::handleEncoderTickLeft() {
  robot.motorLeft.incrementEncoderTickCount();
}

static void Robot::handleEncoderTickRight() {
  robot.motorRight.incrementEncoderTickCount();
  
}

