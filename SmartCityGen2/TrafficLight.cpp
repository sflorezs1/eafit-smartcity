#include "TrafficLight.h"
#include "InfraredSensor.h"
#include <Arduino.h>

TrafficLight::TrafficLight(
  int red,
  int yellow,
  int green,
  LightSensor ls1,
  InfraredSensor pss[],
  Button b1)
  : ls(ls1), 
    pedestrianButton(b1),
    presenceSensors{ pss[0], pss[1], pss[2] }
  {
  redPin = red;
  yellowPin = yellow;
  greenPin = green;

  pinMode(redPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(greenPin, OUTPUT);

  initialize();
}

void TrafficLight::initialize() {
  digitalWrite(redPin, LOW);
  digitalWrite(yellowPin, LOW);
  digitalWrite(greenPin, LOW);
}

void TrafficLight::setGreen() {
  digitalWrite(redPin, LOW);
  digitalWrite(yellowPin, LOW);
  digitalWrite(greenPin, HIGH);
}

void TrafficLight::setYellow() {
  digitalWrite(greenPin, LOW);
  digitalWrite(yellowPin, HIGH);
}

void TrafficLight::setRed() {
  digitalWrite(yellowPin, LOW);
  digitalWrite(redPin, HIGH);
}

void TrafficLight::setBlinking() {
  digitalWrite(redPin, LOW);
  digitalWrite(yellowPin, LOW);
  digitalWrite(greenPin, LOW);
}

bool TrafficLight::isNight() {
  return ls.isNightTime();  // Using isNightTime() method from LightSensor
}

int TrafficLight::getVehicleCount() {
  int count = 0;
  for (unsigned short i = 0; i < 3; i++) {
    count += presenceSensors[i].isVehiclePresent();
  }
  return count;
}
