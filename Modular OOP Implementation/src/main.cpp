#include <Arduino.h>
#include "MotorDriver.h"
#include "LineFollower.h"
#include "ColorSensor.h"
#include "UltrasonicSensor.h"
#include "ServoScanner.h"

// Setup instances
MotorDriver motors(9, 8, 3, 7, 5, 2);
LineFollower lineSensor(A0, A1);
ColorSensor colorSensor(A2, A3, A4, A5, 10);
UltrasonicSensor ultrasonic(13, 12);
ServoScanner scanner(11);

void setup() {
  Serial.begin(9600);
  colorSensor.begin();
}

void loop() {
  scanner.scan(); // rotate ultrasonic
  long distance = ultrasonic.readDistance();
  Serial.print("Distance: "); Serial.println(distance);

  if (distance <= 10 && distance > 0) {
    motors.stop();
    return;
  }

  if (colorSensor.isRed()) {
    Serial.println("Red detected: Stopping");
    motors.stop();
    delay(1000);
    return;
  }

  int state = lineSensor.readState();
  switch (state) {
    case 0b11: motors.moveForward(); break;
    case 0b10: motors.turnLeft(); break;
    case 0b01: motors.turnRight(); break;
    case 0b00: motors.stop(); break;
  }

  delay(100);
}