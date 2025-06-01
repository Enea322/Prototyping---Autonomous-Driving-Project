#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

class UltrasonicSensor {
  int trig, echo;

public:
  UltrasonicSensor(int trig, int echo);
  long readDistance();
};

#endif