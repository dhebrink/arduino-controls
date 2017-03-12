#include "sensor.h"

const int baudCount = 9600;

Sensor sensor1(13, 12);

void setup() {
  sensor1.setUp();
  Serial.begin(baudCount);
}

void loop() {
  int distance = sensor1.getDistance();
  Serial.println(distance);
  delay(250);
}
