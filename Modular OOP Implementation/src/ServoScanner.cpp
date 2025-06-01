#include "ServoScanner.h"
#include <Arduino.h>

ServoScanner::ServoScanner(int pin, int step) : pin(pin), step(step) {}

void ServoScanner::scan() {
  servo.attach(pin);
  for (int angle = 0; angle <= 180; angle += step) {
    servo.write(angle); delay(15);
  }
  for (int angle = 180; angle >= 0; angle -= step) {
    servo.write(angle); delay(15);
  }
}
