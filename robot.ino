#include "robot.h"
#include "motor.h"

Robot::Robot() {}
Robot::~Robot() {}

void Robot::setUp() {
  attachInterrupt(digitalPinToInterrupt(18), handleEncoderTickLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(19), handleEncoderTickRight, RISING);
  
  sensorForward.setUp();
  sensorRight.setUp();
  sensorLeft.setUp();
  motorLeft.setUp();
  motorRight.setUp();
}

void Robot::step() {
  stepNavigation();
  stepObstacleAvoidance();
  stepBumpers();
  stepMotors();
  
  updateOdometry();
  updateHeadingError();
  
  printStats();
  delay(50);
}

void Robot::printStats() {
  Serial.print("Position: ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(theta);
  Serial.print(";  ");
  Serial.print("Heading Error: ");
  Serial.println(headingError);
}

// todo: replace with PID controller
void Robot::stepNavigation() {
  float velocity = 15;
  float omega = 0;

  if (targetDistance < 5) {
    targetVelocityLeft = 0;
    targetVelocityRight = 0;
  } else {
    if (headingError > PI/8) {
      omega = -PI/8;
    } else if (headingError < -PI/8) {
      omega = PI/8;
    }
    
    targetVelocityLeft = (2*velocity + omega*wheelAxelLength)/(2*wheelRadius);
    targetVelocityRight = (2*velocity - omega*wheelAxelLength)/(2*wheelRadius);
  }
}

void Robot::stepObstacleAvoidance() {
  
}

void Robot::stepBumpers() {
  
}

int leftAccumulation, rightAccumulation; // accumulators for motor output values
int leftErrorLast, rightErrorLast; // history values for calculating derivative

// todo: move to PID controller class
void Robot::stepMotors() {
  // PID Controller Gain Parameters
  int Kp = 3; // proportional constant
  int Kd = 3; // derivative constant
  
  int maxOutputValue = 100*256;

  int leftError = (targetVelocityLeft - velocityLeft) * 256;
  leftAccumulation +=  ((leftError / Kp) + ((leftError - leftErrorLast) / Kd));
  leftErrorLast = leftError;
  
  if (leftAccumulation > maxOutputValue) leftAccumulation = maxOutputValue;
  else if (leftAccumulation < -maxOutputValue) leftAccumulation = -maxOutputValue;
  
  int rightError = (targetVelocityRight - velocityRight) * 256;
  rightAccumulation += ((rightError / Kp) + ((rightError - rightErrorLast) / Kd));
  rightErrorLast = rightError;
  
  if (rightAccumulation > maxOutputValue) rightAccumulation = maxOutputValue;
  else if (rightAccumulation < -maxOutputValue) rightAccumulation = -maxOutputValue;
  
  motorRight.step(rightAccumulation/256); 
  motorLeft.step(leftAccumulation/256); 
}

void Robot::updateOdometry() {
  int ticksPerRevolution = 192;
  
  int countLeft = motorLeft.getEncoderTickCount();
  int countRight = motorRight.getEncoderTickCount();
  velocityLeft = countLeft - lastEncoderTickCountLeft;
  velocityRight = countRight - lastEncoderTickCountRight;

  lastEncoderTickCountLeft = countLeft;
  lastEncoderTickCountRight = countRight;

  // distance moved given # of wheel ticks and wheel radius
  float distanceLeft = 2.0 * PI * wheelRadius * ((float)velocityLeft / ticksPerRevolution);
  float distanceRight = 2.0 * PI * wheelRadius * ((float)velocityRight / ticksPerRevolution);
  float distanceCenter = (distanceLeft + distanceRight) / 2.0;

  // trigonometry to determine XY coordinates of center of robot
  x = x + distanceCenter * sin(theta);
  y = y + distanceCenter * cos(theta);
  theta = theta + ((distanceRight - distanceLeft) / wheelAxelLength);

  // keep theta between -360 degrees and +360 degress (makes error calculations much easier)
  if (theta < -TWO_PI) theta += TWO_PI;
  else if (theta > TWO_PI) theta -= TWO_PI;
}

void Robot::updateHeadingError() {
  // error given target and current position
  float errorX = targetX - x;
  float errorY = targetY - y;
  targetDistance = sqrt((errorX*errorX) + (errorY*errorY));
  
  // ensure no division by zero
  if (errorX > 0.00001) targetBearing = PI/2 - atan(errorY/errorX);
  else if (errorX < -0.00001) targetBearing = -PI/2 - atan(errorY/errorX);

  // # of radians robot needs to turn to face target
  headingError = targetBearing - theta;
  if (headingError > PI) headingError -= TWO_PI;
  else if (headingError < -PI) headingError += TWO_PI;
}

void Robot::drive() {
  stepMotors();
}

void Robot::setVelocity(int leftVelocity, int rightVelocity) {
  targetVelocityLeft = leftVelocity;
  targetVelocityRight = rightVelocity;
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

