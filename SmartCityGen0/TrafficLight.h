#ifndef TRAFFIC_LIGHT_H
#define TRAFFIC_LIGHT_H

#include "Button.h"

enum State {
  RED,
  GREEN,
  YELLOW
};

class TrafficLight {
private:
  int redPin;
  int yellowPin;
  int greenPin;

public:
  TrafficLight(
    int red,
    int yellow,
    int green,
    Button b1);
  Button pedestrianButton;
  void initialize();
  void setGreen();
  void setYellow();
  void setRed();
};

#endif
