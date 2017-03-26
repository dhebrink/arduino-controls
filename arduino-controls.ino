#include "robot.h"
#include "test.h"

const Robot robot;
const Test test;

void setup() {
  Serial.begin(9600);
  robot.setUp();
}

void loop() {
  // For testing, put some functionality into a temporary function, then add it
  // here. Uncomment/comment lines you want to test.
  // Un-comment the `return` line to make the robot chill bro

  //return;
  //test.all(robot);
  robot.step();
  
//  robot.setVelocity(10, 10);
//  robot.drive();
//  robot.updateOdometry();
//  robot.printStats();
//  delay(50);
} 
