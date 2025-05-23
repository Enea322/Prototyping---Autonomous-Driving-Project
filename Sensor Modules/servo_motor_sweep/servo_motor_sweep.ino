/*
Servo Motor code v1, where the servo motor is spinning in 180° with an Ultrasonic Sensor attached into it.
*/

// C++ Code
#include <Servo.h>

const int servoPin = 11;
Servo servo;
int angle = 0;
int step = 5; // degrees per step

void setup() {
  servo.attach(servoPin); // Attach servo to pin
}

void loop() {
  // Spin from 0 to 180
  for (angle = 0; angle <= 180; angle += step) {
    servo.write(angle);
    delay(15);
  }

  // Spin from 180 to 0
  for (angle = 180; angle >= 0; angle -= step) {
    servo.write(angle);
    delay(15);
  }
}