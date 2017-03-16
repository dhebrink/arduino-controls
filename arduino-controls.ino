#include "robot.h"

const int baudCount = 9600;
const Robot robot;
const int forwardDistanceThreshold = 50;

bool distanceIsSafe(int givenDistance) {
  return (givenDistance > 0 && givenDistance < forwardDistanceThreshold);
}

void setup() {
  Serial.begin(baudCount);
  robot.setUp();
}

void loop() {
  int distance = robot.sensorForward.getDistance();
  Serial.println(distance);

  if (distanceIsSafe(distance)) {
    robot.moveStop();
    delay(1000);
    int leftDistance = robot.sensorLeft.getDistance();
    if (distanceIsSafe(leftDistance)) {
      robot.moveLeft(90);
    }
    else {
      // Never seems to get hit...?
      robot.moveRight(90);
    }
    delay(1000);
  }
  else {
    robot.moveForward();
  }
}
