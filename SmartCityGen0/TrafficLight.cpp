#include <Arduino.h>
#include "TrafficLight.h"

TrafficLight::TrafficLight(
  int red,
  int yellow,
  int green,
  Button b1)
  : pedestrianButton(b1) {
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
