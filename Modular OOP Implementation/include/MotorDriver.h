#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

class MotorDriver {
  int in1, in2, ena, in3, in4, enb;

public:
  MotorDriver(int in1, int in2, int ena, int in3, int in4, int enb);
  void moveForward(int speed = 100);
  void turnLeft();
  void turnRight();
  void stop();
};

#endif