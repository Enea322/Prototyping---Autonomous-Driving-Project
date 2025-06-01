#ifndef COLORSENSOR_H
#define COLORSENSOR_H

class ColorSensor {
  int s0, s1, s2, s3, out;
  float redAmbient;

public:
  ColorSensor(int s0, int s1, int s2, int s3, int out);
  void begin();
  bool isRed();
};

#endif