#ifndef CO2_SENSOR_H
#define CO2_SENSOR_H

class Co2Sensor {
private:
  int pin;
  float zeroPointVoltage;
  float reactionVoltage;
  float dcGain;
  float co2Curve[3];

public:
  Co2Sensor(int sensorPin, float zeroPoint, float reaction, float gain);
  float GetValue();
};

#endif
