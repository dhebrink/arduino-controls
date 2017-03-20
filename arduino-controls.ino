#include "robot.h"
#include "test.h"

const Robot robot;
const Test test;
const int distanceThreshold = 20;

bool distanceIsSafe(int givenDistance) {
  return (givenDistance > 0 && givenDistance < distanceThreshold);
}

void setup() {
  Serial.begin(9600);
  robot.setUp();
}

void loop() {
  // For testing, put some functionality into a temporary function, then add it
  // here. Uncomment/comment lines you want to test.
  // Un-comment the `return` line to make the robot chill bro

  //return;
  //test.TurningLeft(robot);
  //test.Acceleration(robot);
}
