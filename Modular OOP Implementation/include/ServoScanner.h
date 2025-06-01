#ifndef SERVOSCANNER_H
#define SERVOSCANNER_H
#include <Servo.h>

class ServoScanner {
  Servo servo;
  int pin;
  int step;

public:
  ServoScanner(int pin, int step = 5);
  void scan();
};

#endif
