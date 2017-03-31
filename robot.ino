#include "robot.h"
#include "motor.h"
#include "pid.h"

int waypointCount = 4;
int waypointX[4] = {0, 12, -36, 0};
int waypointY[4] = {0, 72, 36, 0};
int waypointCurrent = 0;
bool navigationComplete = false;

Robot::Robot() {}

Robot::~Robot() {}

void Robot::setUp() {
  attachInterrupt(digitalPinToInterrupt(18), handleEncoderTickLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(19), handleEncoderTickRight, RISING);
  
  regulatorMotorLeft.SetMode(AUTOMATIC);
  regulatorMotorLeft.SetSampleTime(25);
  regulatorMotorLeft.SetOutputLimits(-100, 100);
  
  regulatorMotorRight.SetMode(AUTOMATIC);
  regulatorMotorRight.SetSampleTime(25);
  regulatorMotorRight.SetOutputLimits(-100, 100);
  
  regulatorNavigationOmega.SetMode(AUTOMATIC);
  regulatorNavigationOmega.SetSampleTime(25);
  regulatorNavigationOmega.SetOutputLimits(-6, 6);
  
  sensorForward.setUp();
  sensorRight.setUp();
  sensorLeft.setUp();
  motorLeft.setUp();
  motorRight.setUp();
}

void Robot::readSensors() {
  // ultrasonic sensors
  distanceForward = sensorForward.getDistanceInches();
  distanceLeft = sensorLeft.getDistanceInches();
  distanceRight = sensorRight.getDistanceInches();
}

void Robot::step() {
  readSensors();
  setWaypoint();
  
  stepNavigation();
//  stepWallFollow();
//  stepObstacleAvoidance();
//  stepBumpers();
  stepMotors();
  
  updateOdometry();
  updateHeadingError();
  
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
  Serial.print("Omega: "); 
  Serial.print(omega);
  Serial.print(";  ");
  Serial.print("Desired Theta: ");
  Serial.println(desiredTheta);
//  Serial.print("Target Distance: ");
//  Serial.println(targetDistance);
//  Serial.print("Target Velocity Left: ");
//  Serial.print(targetVelocityLeft);
//  Serial.print(", Target Velocity Right: ");
//  Serial.println(targetVelocityRight);
//  Serial.print("Left PWM: ");
//  Serial.print(motorPwmLeft);
//  Serial.print(", Right PWM: ");
//  Serial.println(motorPwmRight);
//  Serial.print("Sensor Forward: ");
//  Serial.print(distanceForward);
//  Serial.print(", Sensor Left: ");
//  Serial.print(distanceLeft);
//  Serial.print(", Sensor Right: ");
//  Serial.println(distanceRight);
}

void Robot::setWaypoint() {
  bool done = isWaypointComplete();
  if (done == true) {
    if (waypointCurrent + 1 <= waypointCount) {
      waypointCurrent++;
      targetX = waypointX[waypointCurrent];
      targetY = waypointY[waypointCurrent];
    }
  }
}

bool Robot::isNavigationComplete() {
  if (navigationComplete == true) {
    return true;
  } else {
    bool result = (isWaypointComplete() && waypointCurrent >= waypointCount);
    if (result == true) {
      navigationComplete = true;
    }
    return result;
  }
}

bool Robot::isWaypointComplete() {
  return (targetDistance < 2);
}

void Robot::stepNavigation() {
  bool done = isNavigationComplete();
  if (done == true) {
    targetVelocityLeft = 0;
    targetVelocityRight = 0;
  } else {
    regulatorNavigationOmega.Compute();

    float velocity = velocitySlow;
    float _omega = omega;
    float thetaError = desiredTheta - theta;

    if (abs(thetaError) > PI) {
      velocity = 0;
      _omega /= 2;
    } else if (abs(thetaError) > PI / 2) {
      velocity /= 2;
      _omega /= 2;
    } else if (targetDistance < 24) {
      velocity /= 2;
    }
    
    targetVelocityLeft = (2*velocity + _omega*wheelAxelLength)/(2*wheelRadius);
    targetVelocityRight = (2*velocity - _omega*wheelAxelLength)/(2*wheelRadius);
  }
}

bool runWallFollowProgram = false;
int wallFollowProgramStep = 0;
int wallFollowProgramStepCount = -1;
void Robot::stepWallFollow() {
  int enterDistance = 24;
  int exitDistance = 36;

  if (abs(desiredTheta - theta) > PI) {
    if (distanceLeft < enterDistance) {
      runWallFollowProgram = true;
      wallFollowProgramStep = 1;
      wallFollowProgramStepCount = -1;
    } else if (distanceRight < enterDistance) {
      runWallFollowProgram = true;
      wallFollowProgramStep = 1;
      wallFollowProgramStepCount = -1;
    }
  }

  if (runWallFollowProgram == true) {
    if (wallFollowProgramStep == 1) {
      if (distanceLeft < enterDistance / 2) {
        omega = PI / 2;
        targetVelocityLeft = (2*velocitySlow + omega*wheelAxelLength)/(2*wheelRadius);
        targetVelocityRight = (2*velocitySlow - omega*wheelAxelLength)/(2*wheelRadius);
      } else if (distanceLeft >= enterDistance / 2) {
        wallFollowProgramStep = 2;
      } else if (distanceRight < enterDistance / 2) {
        omega = -PI / 2;
        targetVelocityLeft = (2*velocitySlow + omega*wheelAxelLength)/(2*wheelRadius);
        targetVelocityRight = (2*velocitySlow - omega*wheelAxelLength)/(2*wheelRadius);
      } else if (distanceRight >= enterDistance / 2) {
        wallFollowProgramStep = 2;
      }
    } else if (wallFollowProgramStep == 2) {
      wallFollowProgramStepCount++;
      targetVelocityLeft = velocitySlow;
      targetVelocityRight = velocitySlow;

      if (wallFollowProgramStepCount >= 20) {
        runWallFollowProgram = false;
        wallFollowProgramStep = 0;
        wallFollowProgramStepCount = -1;
      }
    }
  }
}

int FORWARD_DETECT = 0;
int LEFT_DETECT = 1;
int RIGHT_DETECT = 2;
void Robot::stepObstacleAvoidance() {
  // classify detections
  int detectType = -1;
  if (distanceLeft < 12) {
    detectType = LEFT_DETECT;
  } else if (distanceRight < 12) {
    detectType = RIGHT_DETECT;
  } else if (distanceForward < 12) {
    detectType = FORWARD_DETECT;
  }

  // act on detection
  if (detectType == LEFT_DETECT) {
    // half speed, right turn
    omega = PI;
    targetVelocityRight = (-omega*wheelAxelLength)/(2*wheelRadius);
    targetVelocityLeft = (omega*wheelAxelLength)/(2*wheelRadius);
  } else if (detectType == RIGHT_DETECT) {
    // half speed, left turn
    omega = -PI;
    targetVelocityRight = (-omega*wheelAxelLength)/(2*wheelRadius);
    targetVelocityLeft = (omega*wheelAxelLength)/(2*wheelRadius);
  } else if (detectType == FORWARD_DETECT) {
    // zero speed, keep turning in same direction
    targetVelocityLeft = -velocitySlow;
    targetVelocityRight = velocitySlow;
  }
}

bool runBumperProgram = false;
int bumperProgramStep = 0;
int bumperProgramStepCount = -1;
void Robot::stepBumpers() {
  if (distanceForward < 5 || distanceLeft < 5 || distanceRight < 5) {
    runBumperProgram = true;
    bumperProgramStep = 1;
    bumperProgramStepCount = -1;
  }

  if (runBumperProgram == true) {
    if (bumperProgramStep == 1) {
      targetVelocityLeft = -velocitySlow / 2;
      targetVelocityRight = -velocitySlow / 2;
  
      bumperProgramStepCount++;
      if (bumperProgramStepCount >= 10) {
        bumperProgramStep = 2;
        bumperProgramStepCount = -1;
      }
    } else if (bumperProgramStep == 2) {
      if (distanceRight > distanceLeft + 5) {
        omega = PI;
        targetVelocityRight = (-omega*wheelAxelLength)/(2*wheelRadius);
        targetVelocityLeft = (omega*wheelAxelLength)/(2*wheelRadius);
      } else if (distanceLeft > distanceRight + 5) {
        omega = -PI;
        targetVelocityRight = (-omega*wheelAxelLength)/(2*wheelRadius);
        targetVelocityLeft = (omega*wheelAxelLength)/(2*wheelRadius);
      } else {
        omega = PI;
        targetVelocityRight = (-omega*wheelAxelLength)/(2*wheelRadius);
        targetVelocityLeft = (omega*wheelAxelLength)/(2*wheelRadius);
      }
  
      bumperProgramStepCount++;
      if (bumperProgramStepCount >= 20) {
        bumperProgramStep = 0;
        bumperProgramStepCount = -1;
        runBumperProgram = false;
      }
    }
  }
}

void Robot::stepMotors() {
  if (targetVelocityLeft == 0) {
    motorLeft.stop();
  } else {
    regulatorMotorLeft.Compute();
    motorLeft.step(motorPwmLeft);
  }

  if (targetVelocityRight == 0) {
    motorRight.stop();
  } else {
    regulatorMotorRight.Compute();
    motorRight.step(motorPwmRight);
  }
}

void Robot::updateOdometry() {
  int ticksPerRevolution = 192;

  // change in ticks from last check
  int tickDeltaLeft = motorLeft.getEncoderTickCountDelta();
  int tickDeltaRight = motorRight.getEncoderTickCountDelta();

  // distance moved given # of wheel ticks and wheel radius
  float distanceLeft = 2.0 * PI * wheelRadius * ((float)tickDeltaLeft / ticksPerRevolution);
  float distanceRight = 2.0 * PI * wheelRadius * ((float)tickDeltaRight / ticksPerRevolution);
  float distanceCenter = (distanceLeft + distanceRight) / 2.0;

  // determine XY coordinates of center of robot and theta (angle)
  x = x + distanceCenter * sin(theta);
  y = y + distanceCenter * cos(theta);
  theta -= (distanceRight - distanceLeft) / wheelAxelLength;

  // keep theta between -180 degrees and +180 degrees (makes error calculations much easier)
  if (theta < -TWO_PI * 2) theta += TWO_PI;
  else if (theta > TWO_PI * 2) theta -= TWO_PI;
}

void Robot::updateHeadingError() {
  // error given target and current position
  float errorX = targetX - x;
  float errorY = targetY - y;
  targetDistance = sqrt((errorX*errorX) + (errorY*errorY));
  desiredTheta = PI/2 - atan2(errorY, errorX);

  //
  if (desiredTheta - theta > PI && abs(theta + TWO_PI - desiredTheta) < abs(desiredTheta - theta)) {
    theta += TWO_PI;
  } else if (theta - desiredTheta > PI && abs(theta + TWO_PI - desiredTheta) > abs(desiredTheta - theta)) {
    theta -= TWO_PI;
  }

  // # of radians robot needs to turn to face target
  headingError = abs(desiredTheta - theta);
  if (headingError > TWO_PI) headingError = headingError - TWO_PI;
  else if (headingError < -TWO_PI) headingError = headingError + TWO_PI;
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
  } else {
     robot.motorLeft.incrementEncoderTickCount();
  }
}

static void Robot::handleEncoderTickRight() {
  if (robot.motorRight.isMovingForward == true) {
    robot.motorRight.incrementEncoderTickCount();
  } else if (robot.motorRight.isMovingBackward == true) {
    robot.motorRight.decrementEncoderTickCount();
  } else {
    robot.motorRight.decrementEncoderTickCount();
  }
}

