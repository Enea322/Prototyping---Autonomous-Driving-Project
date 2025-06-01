#include "MotorDriver.h"
#include <Arduino.h>

MotorDriver::MotorDriver(int in1, int in2, int ena, int in3, int in4, int enb)
  : in1(in1), in2(in2), ena(ena), in3(in3), in4(in4), enb(enb) {
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT); pinMode(ena, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT); pinMode(enb, OUTPUT);
}

void MotorDriver::moveForward(int speed) {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW); analogWrite(ena, speed);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW); analogWrite(enb, speed);
}

void MotorDriver::turnLeft() {
  digitalWrite(in1, LOW); digitalWrite(in2, LOW); analogWrite(ena, 0);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW); analogWrite(enb, 100);
}

void MotorDriver::turnRight() {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW); analogWrite(ena, 100);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW); analogWrite(enb, 0);
}

void MotorDriver::stop() {
  digitalWrite(in1, LOW); digitalWrite(in2, LOW); analogWrite(ena, 0);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW); analogWrite(enb, 0);
}
