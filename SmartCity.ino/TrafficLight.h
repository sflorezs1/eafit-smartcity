#ifndef TRAFFIC_LIGHT_H
#define TRAFFIC_LIGHT_H

#include "LightSensor.h"
#include "InfraredSensor.h"
#include "Button.h"

enum State {
  RED,
  GREEN,
  YELLOW,
  BLINKING_YELLOW
};

class TrafficLight {
private:
  int redPin;
  int yellowPin;
  int greenPin;
  LightSensor ls;
  InfraredSensor presenceSensors[3];

public:
  TrafficLight(
    int red,
    int yellow,
    int green,
    LightSensor ls1,
    InfraredSensor pss[],
    Button b1
  );
  Button pedestrianButton;
  int getVehicleCount();
  void initialize();
  void setGreen();
  void setYellow();
  void setRed();
  void setBlinking();
  bool isNight();
};

#endif
