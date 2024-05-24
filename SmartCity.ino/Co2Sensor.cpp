#include "Co2Sensor.h"
#include <Arduino.h>
#include <math.h>

Co2Sensor::Co2Sensor(int sensorPin, float zeroPoint, float reaction, float gain) {
  pin = sensorPin;
  zeroPointVoltage = zeroPoint;
  reactionVoltage = reaction;
  dcGain = gain;
  co2Curve[0] = 2.602;
  co2Curve[1] = zeroPointVoltage;
  co2Curve[2] = reaction / (2.602 - 3);
  pinMode(pin, INPUT);
}

float Co2Sensor::GetValue() {
  float volts = analogRead(pin) * 5.0 / 1023.0;  // Convert CO2 ADC to volts
  if (volts / dcGain >= zeroPointVoltage) {
    return -1;  // Indicate an error reading CO2
  } else {
    return pow(10, ((volts / dcGain) - co2Curve[1]) / co2Curve[2] + co2Curve[0]);
  }
}
