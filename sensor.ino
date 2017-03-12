#include "sensor.h"

const double speedOfSound = 0.034;


Sensor::Sensor(int triggerPin, int echoPin) {
    this->triggerPin = triggerPin;
    this->echoPin = echoPin;
}

Sensor::~Sensor() {
}

void Sensor::setUp() {
    pinMode(this->triggerPin, OUTPUT);
    pinMode(this->echoPin, INPUT);
}

int Sensor::getDistance() {
  // Making sure the trigPin is clear for use
  digitalWrite(this->triggerPin, LOW);
  delayMicroseconds(2);

  // Generate ultrasound wave
  digitalWrite(this->triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(this->triggerPin, LOW);

  // "HIGH" given meaning it will wait for HIGH voltage to begin
  // and time until the HIGH signal ends.
  int duration = pulseIn(this->echoPin, HIGH);

  // Multiply duration by speed of sound factor,
  // then divide by 2 since it count the travel time
  // to and from the endpoint.
  int distance = (duration * speedOfSound) / 2;
  return distance;
}
