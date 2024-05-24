#include "Button.h"
#include <Arduino.h>

Button::Button(int pin, unsigned long debounceDelay) {
    this->pin = pin;
    this->debounceDelay = debounceDelay;
    pinMode(pin, INPUT);
    wasPressedInCycle = false;
    lastDebounceTime = 0;
}

void Button::update() {
  bool reading = digitalRead(pin);
  if ((reading == HIGH) && ((millis() - lastDebounceTime) > debounceDelay)) {
    Serial.print("Button changed: ");
    wasPressedInCycle = reading;
    Serial.println(wasPressedInCycle);
    lastDebounceTime = millis();
  }
}

bool Button::wasPressed() {
  return wasPressedInCycle;
}

void Button::reset() {
  wasPressedInCycle = false;
}
