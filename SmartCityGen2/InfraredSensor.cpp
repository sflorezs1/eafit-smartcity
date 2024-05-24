#include "InfraredSensor.h"
#include <Arduino.h>

InfraredSensor::InfraredSensor(int sensorPin) {
  pin = sensorPin;
  pinMode(pin, INPUT);
}

bool InfraredSensor::isVehiclePresent() {
  return digitalRead(pin) == LOW;
}
