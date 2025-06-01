#include "ColorSensor.h"
#include <Arduino.h>

ColorSensor::ColorSensor(int s0, int s1, int s2, int s3, int out)
  : s0(s0), s1(s1), s2(s2), s3(s3), out(out), redAmbient(0) {}

void ColorSensor::begin() {
  pinMode(s0, OUTPUT); pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT); pinMode(s3, OUTPUT);
  pinMode(out, INPUT);

  digitalWrite(s0, LOW); digitalWrite(s1, HIGH);

  float redSum = 0;
  for (int i = 0; i < 10; i++) {
    digitalWrite(s2, LOW); digitalWrite(s3, LOW);
    long edge = pulseIn(out, HIGH) + pulseIn(out, LOW);
    redSum += 1.0 / (edge / 1000000.0);
  }
  redAmbient = redSum / 10.0;
}

bool ColorSensor::isRed() {
  float redSum = 0;
  for (int i = 0; i < 10; i++) {
    digitalWrite(s2, LOW); digitalWrite(s3, LOW);
    long edge = pulseIn(out, HIGH) + pulseIn(out, LOW);
    redSum += 1.0 / (edge / 1000000.0);
  }
  float redAvg = redSum / 10.0;
  return redAvg > redAmbient;
}