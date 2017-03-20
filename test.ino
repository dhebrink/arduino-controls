#include "robot.h"

static void Test::TurningLeft(Robot robot) {
  int distance = robot.sensorForward.getDistance();
  Serial.println(distance);

  if (distanceIsSafe(distance)) {
    robot.moveStop();
    delay(1000);
    int leftDistance = robot.sensorLeft.getDistance();
    if (distanceIsSafe(leftDistance)) {
      robot.turnLeft(90);
    }
    else {
      // Never seems to get hit...?
      robot.turnRight(90);
    }
    delay(1000);
  }
  else {
    robot.moveForward();
  }
}

static void Test::Acceleration(Robot robot) {
  int startingSpeed = 10;
  int maxSpeed = 100;
  robot.moveStop();
  delay(2000);
  // Speed up until we get to max speed
  robot.accelerate(maxSpeed);
  // Wait for a couple seconds, should hold max speed
  delay(4000);
  // Slow back down to original speed
  robot.decelerate(startingSpeed);
  // Wait for a couple seconds, should hold slow speed
  delay(4000);
}
