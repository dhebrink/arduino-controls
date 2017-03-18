#include "robot.h"

const Robot robot;
const int distanceThreshold = 20;

bool distanceIsSafe(int givenDistance) {
  return (givenDistance > 0 && givenDistance < distanceThreshold);
}

void testTurningLeft() {
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

void testAcceleration() {
  int startingSpeed = 10;
  int maxSpeed = 100;
  // Speed up until we get to max speed
  robot.accelerate(maxSpeed);
  // Wait for a couple seconds, should hold max speed
  delay(4000);
  // Slow back down to original speed
  robot.decelerate(startingSpeed);
  // Wait for a couple seconds, should hold slow speed
  delay(4000);
}

void setup() {
  Serial.begin(9600);
  robot.setUp();
}

void loop() {
  // For testing, put some functionality into a temporary function, then add it
  // here. Uncomment/comment lines you want to test.
  // Un-comment the `return` line to make the robot chill bro

  //return
  //testTurningLeft();
  testAcceleration();
}
