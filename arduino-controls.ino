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
  
  robot.printStats();
  for (int i = 0; i < 10; i++) {
    robot.step();
  }
} 
