#include "robot.h"

Motor::Motor(int pinEnable, int pinDrive1, int pinDrive2) {
  this->motorSpeed = 255;
  this->pinEnable = pinEnable;
  this->pinDrive1 = pinDrive1;
  this->pinDrive2 = pinDrive2;
}

Motor::~Motor() {
  this->stop();
}

void Motor::setUp() {
  pinMode(pinEnable, OUTPUT);
  pinMode(pinDrive1, OUTPUT);
  pinMode(pinDrive2, OUTPUT);
  analogWrite(pinEnable, motorSpeed);
}

void Motor::forward() {
  analogWrite(pinEnable, motorSpeed);
  digitalWrite(pinDrive1, HIGH);
  digitalWrite(pinDrive2, LOW);
}

void Motor::backward() {
  analogWrite(pinEnable, motorSpeed);
  digitalWrite(pinDrive1, LOW);
  digitalWrite(pinDrive2, HIGH);
}

void Motor::stop() {
  analogWrite(pinEnable, LOW);
  digitalWrite(pinDrive1, LOW);
  digitalWrite(pinDrive2, LOW);
}
