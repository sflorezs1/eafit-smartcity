#ifndef LIGHT_SENSOR_H
#define LIGHT_SENSOR_H

class LightSensor {
private:
  int analogPin;

public:
  LightSensor(int pin);
  int read();
  bool isNightTime();
};

#endif
