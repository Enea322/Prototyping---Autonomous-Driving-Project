#include "LineFollower.h"
#include <Arduino.h>

LineFollower::LineFollower(int leftPin, int rightPin)
  : leftPin(leftPin), rightPin(rightPin) {
  pinMode(leftPin, INPUT);
  pinMode(rightPin, INPUT);
}

int LineFollower::readState() {
  int left = digitalRead(leftPin);
  int right = digitalRead(rightPin);
  return (left << 1) | right;
}