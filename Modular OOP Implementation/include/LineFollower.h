#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

class LineFollower {
  int leftPin, rightPin;

public:
  LineFollower(int leftPin, int rightPin);
  int readState();
};

#endif