#include "robot.h"

static void Test::all(Robot robot) {
  Serial.println("Testing begin");
  Serial.println("Testing turning movement");
  turning(robot);
  Serial.println("Testing forward/backward movement");
  forwardBackward(robot);
  Serial.println("Testing completed");
}

static void Test::turning(Robot robot) {
  for (int i = 0; i < 80; i++) {
    robot.setVelocity(10, -10);
    robot.drive();
    delay(50);
  }
  
  for (int i = 0; i < 80; i++) {
    robot.setVelocity(-10, 10);
    robot.drive();
    delay(50);
  }
}

static void Test::forwardBackward(Robot robot) {
  for (int i = 0; i < 80; i++) {
    robot.setVelocity(10, 10);
    robot.drive();
    delay(50);
  }
  
  for (int i = 0; i < 80; i++) {
    robot.setVelocity(-10, -10);
    robot.drive();
    delay(50);
  }
}
