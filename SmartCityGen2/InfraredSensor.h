#ifndef INFRARED_SENSOR_H
#define INFRARED_SENSOR_H

class InfraredSensor {
private:
  int pin;

public:
  InfraredSensor(int sensorPin);
  bool isVehiclePresent();
};

#endif
