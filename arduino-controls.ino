#include "robot.h"

const int baudCount = 9600;
const Robot robot;


void setup() {
  Serial.begin(baudCount);
  robot.setUp();
}

void loop() {
  int distance = robot.sensorForward.getDistance();
  Serial.println(distance);
  delay(250);
}
