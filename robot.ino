#include "robot.h"
#include "motor.h"
#include "pid.h"

Robot::Robot() {}

Robot::~Robot() {}

void Robot::setUp() {
  attachInterrupt(digitalPinToInterrupt(18), handleEncoderTickLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(19), handleEncoderTickRight, RISING);
  
  regulatorMotorLeft.SetMode(AUTOMATIC);
  regulatorMotorLeft.SetSampleTime(25);
  regulatorMotorLeft.SetOutputLimits(-255, 255);
  
  regulatorMotorRight.SetMode(AUTOMATIC);
  regulatorMotorRight.SetSampleTime(25);
  regulatorMotorRight.SetOutputLimits(-255, 255);
  
  regulatorNavigationOmega.SetMode(AUTOMATIC);
  regulatorNavigationOmega.SetSampleTime(25);
  regulatorNavigationOmega.SetOutputLimits(-TWO_PI, TWO_PI);
  
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
  targetVelocityLeft = 10;
  targetVelocityRight = 10;

  readSensors();
  
  stepNavigation();
  stepWallFollow();
  stepObstacleAvoidance();
  stepBumpers();
  stepMotors();
  
  updateOdometry();
  updateHeadingError();
  
  printStats();
  
  delay(50);
}

void Robot::printStats() {
//  Serial.print("Position: ");
//  Serial.print(x);
//  Serial.print(", ");
//  Serial.print(y);
//  Serial.print(", ");
//  Serial.print(theta);
//  Serial.print(";  ");
//  Serial.print("Omega: "); 
//  Serial.print(omega);
//  Serial.print(";  ");
//  Serial.print("Desired Theta: ");
//  Serial.println(desiredTheta);
//  
//  Serial.print("Target Velocity Left: ");
//  Serial.print(targetVelocityLeft);
//  Serial.print(", Target Velocity Right: ");
//  Serial.println(targetVelocityRight);
//  
//  Serial.print("Sensor Forward: ");
//  Serial.print(distanceForward);
//  Serial.print(", Sensor Left: ");
//  Serial.print(distanceLeft);
//  Serial.print(", Sensor Right: ");
//  Serial.println(distanceRight);
}

// todo: replace with PID controller
bool done = false;
void Robot::stepNavigation() {
  if (done == true) {
    targetVelocityLeft = 0;
    targetVelocityRight = 0;
    return;
  }
  
  if (targetDistance < 5) {
    done = true;
  } else {
    float velocity = 15;
    regulatorNavigationOmega.Compute();
    targetVelocityLeft = (2*velocity + omega*wheelAxelLength)/(2*wheelRadius);
    targetVelocityRight = (2*velocity - omega*wheelAxelLength)/(2*wheelRadius);
  }
}

bool runWallFollowProgram = false;
int wallFollowProgramStep = 0;
int wallFollowProgramStepCount = -1;
void Robot::stepWallFollow() {
  int enterDistance = 24;
  int exitDistance = 36;

  if (distanceLeft < enterDistance) {
    runWallFollowProgram = true;
    wallFollowProgramStep = 1;
    wallFollowProgramStepCount = -1;
  } else if (distanceRight < enterDistance) {
    runWallFollowProgram = true;
    wallFollowProgramStep = 1;
    wallFollowProgramStepCount = -1;
  }

  if (runWallFollowProgram == true) {
    if (wallFollowProgramStep == 1) {
      if (distanceLeft < enterDistance / 2) {
        float velocity = 10;
        omega = PI / 2;
        targetVelocityLeft = (2*velocity + omega*wheelAxelLength)/(2*wheelRadius);
        targetVelocityRight = (2*velocity - omega*wheelAxelLength)/(2*wheelRadius);
      } else if (distanceLeft >= enterDistance / 2) {
        wallFollowProgramStep = 2;
      } else if (distanceRight < enterDistance / 2) {
        float velocity = 10;
        omega = -PI / 2;
        targetVelocityLeft = (2*velocity + omega*wheelAxelLength)/(2*wheelRadius);
        targetVelocityRight = (2*velocity - omega*wheelAxelLength)/(2*wheelRadius);
      } else if (distanceRight >= enterDistance / 2) {
        wallFollowProgramStep = 2;
      }
    } else if (wallFollowProgramStep == 2) {
      wallFollowProgramStepCount++;
      targetVelocityLeft = 10;
      targetVelocityRight = 10;

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
    targetVelocityLeft = -10;
    targetVelocityRight = 10;
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
      targetVelocityLeft = -10;
      targetVelocityRight = -10;
  
      bumperProgramStepCount++;
      if (bumperProgramStepCount >= 15) {
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
  regulatorMotorRight.Compute();
  regulatorMotorLeft.Compute();
  motorRight.step(motorPwmRight);
  motorLeft.step(motorPwmLeft);
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
  theta = theta - (((distanceRight - distanceLeft) / wheelAxelLength) * PI);

  // keep theta between -360 degrees and +360 degrees (makes error calculations much easier)
  if (theta < -TWO_PI) theta += TWO_PI;
  else if (theta > TWO_PI) theta -= TWO_PI;
}

void Robot::updateHeadingError() {
  // error given target and current position
  float errorX = targetX - x;
  float errorY = targetY - y;
  targetDistance = sqrt((errorX*errorX) + (errorY*errorY));
  
  // ensure no division by zero
  if (errorX > 0.00001) desiredTheta = PI/2 - atan(errorY/errorX);
  else if (errorX < -0.00001) desiredTheta = -PI/2 - atan(errorY/errorX);

  // # of radians robot needs to turn to face target
  headingError = abs(desiredTheta - theta);
  if (headingError > TWO_PI) headingError = abs(headingError - TWO_PI);
  else if (headingError < -TWO_PI) headingError = abs(headingError + TWO_PI);
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

