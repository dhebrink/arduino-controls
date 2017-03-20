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

bool targetAcquired = false;
void Robot::stepAutonomously() {
  int targetX = 36; // inches
  int targetY = 0; // inches
  int errorMargin = 2; // inches

  if (targetAcquired == false) {
    if (x < targetX - errorMargin) {
      moveForward(75);
    } else if (x > targetX - errorMargin && x < targetX + errorMargin) {
      targetAcquired = true;
      moveStop();
    }
  } else {
    if (abs(phi) > PI - 0.1 && abs(phi) < PI + 0.1) {
      if (x > -errorMargin) {
        moveForward(75);
      } else {
        moveStop();
        delay(60000);
      }
    } else if (phi >= 0 && phi < PI + 0.1) {
      turnLeft(1);
    } else if (phi >= PI + 0.15) {
      turnRight(1);
    } else if (phi < 0 && phi > PI - 0.1) {
      turnRight(1);
    } else if (phi < PI - 0.1) {
      turnLeft(1);
    }
  }
  
  updateOdometry();
  delay(50);
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
  updateOdometry();
}

void Robot::moveBackward(int speed) {
  motorLeft.backward(speed);
  motorRight.backward(speed);
  updateOdometry();
}

void Robot::turnLeft(int degreeOfTurn, int speed) {
  motorLeft.backward(speed);
  motorRight.forward(speed);
  // for time-delayed turning, 180 deg is roughly 1000 milliseconds
  int waitDuration = int((degreeOfTurn / 180.0) * 1000);
  Serial.println("Turning LEFT for " + String(waitDuration));
  delay(waitDuration);
//  this->moveStop();
  updateOdometry();
}

void Robot::turnRight(int degreeOfTurn, int speed) {
  motorLeft.forward();
  motorRight.backward();
  int waitDuration = int((degreeOfTurn / 180.0) * 1000);
  Serial.println("Turning RIGHT for " + String(waitDuration));
  delay(waitDuration);
//  this->moveStop();
  updateOdometry();
}

void Robot::updateOdometry() {
  int ticksPerRevolution = 192;
  float wheelRadius = 2.5; // inches
  float wheelAxelLength = 14.5; // inches
  
  long long countLeft = motorLeft.getEncoderTickCount();
  long long countRight = motorRight.getEncoderTickCount();
  long long deltaLeft = countLeft - lastEncoderTickCountLeft;
  long long deltaRight = countRight - lastEncoderTickCountRight;

  lastEncoderTickCountLeft = countLeft;
  lastEncoderTickCountRight = countRight;

  float distanceLeft = 2.0 * PI * wheelRadius * ((float)deltaLeft / ticksPerRevolution);
  float distanceRight = 2.0 * PI * wheelRadius * ((float)deltaRight / ticksPerRevolution);
  float distanceCenter = (distanceLeft + distanceRight) / 2.0;

  x = x + distanceCenter * cos(phi);
  y = y + distanceCenter * sin(phi);
  phi = phi + ((distanceRight - distanceLeft) / wheelAxelLength);
}

void Robot::moveStop() {
  motorLeft.stop();
  motorRight.stop();
  Serial.print("Right: ");
  Serial.print((int)motorRight.getEncoderTickCount());
  Serial.print(", Left: ");
  Serial.print((int)motorLeft.getEncoderTickCount());
  Serial.print(", x: ");
  Serial.print(x);
  Serial.print(", y: ");
  Serial.print(y);
  Serial.print(", phi: ");
  Serial.println(phi);
}

static void Robot::handleEncoderTickLeft() {
  if (robot.motorLeft.isMovingForward == true) {
    robot.motorLeft.incrementEncoderTickCount();
  } else if (robot.motorLeft.isMovingBackward == true) {
    robot.motorLeft.decrementEncoderTickCount();
  }
}

static void Robot::handleEncoderTickRight() {
  if (robot.motorRight.isMovingForward == true) {
    robot.motorRight.incrementEncoderTickCount();
  } else if (robot.motorRight.isMovingBackward == true) {
    robot.motorRight.decrementEncoderTickCount();
  }
}

