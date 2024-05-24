#include "TrafficLight.h"
#include <Arduino.h>

LightSensor::LightSensor(int pin) {
  analogPin = pin;
}

int LightSensor::read() {
  return analogRead(analogPin);
}

bool LightSensor::isNightTime() {
  return read() < 300;
}