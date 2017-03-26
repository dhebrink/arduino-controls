#include "sensor.h"

const float tempOfAirCelcius = 23;
const float speedOfSoundMS = 331.3 + (0.606 * tempOfAirCelcius); // meters per second

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

int Sensor::getDistanceInches() {
  // Making sure the trigPin is clear for use
  digitalWrite(this->triggerPin, LOW);
  delayMicroseconds(2);

  // Generate ultrasound wave
  digitalWrite(this->triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(this->triggerPin, LOW);

  // "HIGH" given meaning it will wait for HIGH voltage to begin
  // and time until the HIGH signal ends.
  int duration = pulseIn(this->echoPin, HIGH, 25000);
  if (duration == 0) {
    duration = 23000;
  }

  // Multiply duration by speed of sound factor,
  // then divide by 2 since it count the travel time
  // to and from the endpoint.
  float distanceCm = duration / 20000.0 * speedOfSoundMS;
  int distanceInches = distanceCm * 0.393701;
  return distanceInches;
}
