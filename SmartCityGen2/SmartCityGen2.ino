#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include "TrafficLight.h"
#include "LightSensor.h"
#include "InfraredSensor.h"
#include "Co2Sensor.h"
#include "Button.h"
#include <ArduinoJson.h>

#define LDR1 0  // LDR Light sensor from traffic light 1 connected in pin A0
#define LDR2 1  // LDR Light sensor from traffic light 2 connected in pin A1
#define P1 37   // Botón de semáforo 1 conectado en pin 37
#define P2 36   // Botón de semáforo 2 conectado en pin 36
#define LR1 22  // Luz roja de semáforo 1 conectada en pin 22
#define LY1 23  // Luz amarilla de semáforo 1 conectada en pin 23
#define LG1 24  // Luz verde de semáforo 1 conectada en pin 24
#define LR2 25  // Luz roja de semáforo 2 conectada en pin 25
#define LG2 27  // Luz verde de semáforo 2 conectada en pin 27
#define LY2 26  // Luz amarilla de semáforo 2 conectada en pin 26
#define CO2 2   // CO2 sensor connected in pin A3

#define CNY1 35  // Infrared sensor 1 in traffic light 1 connected in pin 35
#define CNY2 34  // Infrared sensor 2 in traffic light 1 connected in pin 34
#define CNY3 33  // Infrared sensor 3 in traffic light 1 connected in pin 33
#define CNY4 32  // Infrared sensor 4 in traffic light 2 connected in pin 32
#define CNY5 31  // Infrared sensor 5 in traffic light 2 connected in pin 31
#define CNY6 30  // Infrared sensor 6 in traffic light 2 connected in pin 30

unsigned long GREEN_TIME1 = 3500;       // Green time for light 1
unsigned long YELLOW_TIME1 = 500;       // Yellow time for light 1
unsigned long RED_TIME1 = 4000;       // Red time for light 1
unsigned long GREEN_TIME2 = 3500;       // Green time for light 2
unsigned long YELLOW_TIME2 = 500;       // Yellow time for light 2
unsigned long RED_TIME2 = 4000;       // Red time for light 2
unsigned long BLINKING_DURATION = 500;  // Blinking duration
unsigned long PEDESTRIAN_REDUCTION = 3500;
float DIFF1_MULT = 1.1;
float DIFF2_MULT = 1.25;
float DIFF3_MULT = 1.5;

#define PEDESTRIAN_DEBOUNCE 2000

#define DEBOUNCE_DELAY 500       // Debounce delay in milliseconds
#define LCD_UPDATE_INTERVAL 500  // Update LCD every 100 milliseconds
#define SERIAL_UPDATE_INTERVAL 10000

byte sunIcon[8] = {
  0b00000,
  0b10101,
  0b01110,
  0b11111,
  0b01110,
  0b10101,
  0b00000,
  0b00000
};

byte moonIcon[8] = {
  0b00000,
  0b01110,
  0b11000,
  0b11000,
  0b11000,
  0b01110,
  0b00000,
  0b00000
};

byte boxEmpty[8] = {
  0b11111,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b11111,
  0b00000
};

byte boxRed[8] = {
  0b11111,
  0b10001,
  0b10101,
  0b10101,
  0b10101,
  0b10001,
  0b11111,
  0b00000
};

byte boxYellow[8] = {
  0b11111,
  0b10001,
  0b10001,
  0b11011,
  0b11011,
  0b10001,
  0b11111,
  0b00000
};

byte boxGreen[8] = {
  0b11111,
  0b10001,
  0b10001,
  0b10101,
  0b10101,
  0b10001,
  0b11111,
  0b00000
};

byte boxCarDetected[8] = {
  0b11111,
  0b10001,
  0b10101,
  0b10101,
  0b10101,
  0b10001,
  0b11111,
  0b00000
};

// Initialize the LCD
LiquidCrystal_I2C lcd(0x27, 16, 4);  // Set the LCD address to 0x27 for a 16 chars and 4 line display

LightSensor ls1(LDR1);
LightSensor ls2(LDR2);

InfraredSensor is1l1(CNY1);
InfraredSensor is2l1(CNY2);
InfraredSensor is3l1(CNY3);
InfraredSensor is1l2(CNY4);
InfraredSensor is2l2(CNY5);
InfraredSensor is3l2(CNY6);

InfraredSensor issl1[3] = { is1l1, is2l1, is3l1 };
InfraredSensor issl2[3] = { is1l2, is2l2, is3l2 };

Button button1(P2, DEBOUNCE_DELAY);
Button button2(P1, DEBOUNCE_DELAY);

TrafficLight light1(LR1, LY1, LG1, ls1, issl1, button1);  // Semáforo 1
TrafficLight light2(LR2, LY2, LG2, ls2, issl2, button2);  // Semáforo 2

// Valores constantes proporcionados
const float DC_GAIN = 8.5;
const float ZERO_POINT_VOLTAGE = 0.265;
const float REACTION_VOLTAGE = 0.059;
Co2Sensor co2Sensor(CO2, ZERO_POINT_VOLTAGE, REACTION_VOLTAGE, DC_GAIN);

enum TrafficLightState {
  GREEN1_RED2,
  YELLOW1_RED2,
  RED1_GREEN2,
  RED1_YELLOW2,
  BLINKING
};

TrafficLightState currentState;
TrafficLightState lastState;  // Variable to store the last state
unsigned long previousMillis = 0;
unsigned long previousLCDMillis = 0;
unsigned long previousSerialMillis = 0;
unsigned long stateDuration = 0;
bool blinkingState = false;
bool pedestrianDebounce = false;

void updateLCD(unsigned long currentMillis) {
  if (currentMillis - previousLCDMillis < LCD_UPDATE_INTERVAL) return;

  previousLCDMillis = currentMillis;

  lcd.setCursor(0, 0);
  lcd.print("s: ");
  switch (currentState) {
    case GREEN1_RED2:
      lcd.print("G1_R2   ");
      break;
    case YELLOW1_RED2:
      lcd.print("Y1_R2   ");
      break;
    case RED1_GREEN2:
      lcd.print("R1_G2   ");
      break;
    case RED1_YELLOW2:
      lcd.print("R1_Y2   ");
      break;
    case BLINKING:
      lcd.print("BLINK   ");
      break;
  }
  lcd.print(" | d: ");
  lcd.print(stateDuration);

  lcd.setCursor(0, 1);
  lcd.print("count: ");
  lcd.print(light1.getVehicleCount());
  lcd.print(" | ");
  lcd.print(light2.getVehicleCount());
  lcd.setCursor(7, 2);
  lcd.write(light1.isNight() ? byte(1) : byte(0));
  lcd.print(" | ");
  lcd.write(light2.isNight() ? byte(1) : byte(0));
  lcd.setCursor(0, 3);
  lcd.print("co2: ");
  float co2 = co2Sensor.GetValue();
  lcd.setCursor(5, 3);
  lcd.print(co2);

  // Display the traffic lights on the LCD
  byte light1Green = (currentState == GREEN1_RED2) ? 5 : 2;
  byte light1Yellow = (currentState == YELLOW1_RED2) ? 4 : 2;
  byte light1Red = (currentState == RED1_YELLOW2 || currentState == RED1_GREEN2) ? 3 : 2;
  
  byte light2Green = (currentState == RED1_GREEN2) ? 5 : 2;
  byte light2Yellow = (currentState == RED1_YELLOW2) ? 4 : 2;
  byte light2Red = (currentState == GREEN1_RED2 || currentState == YELLOW1_RED2) ? 3 : 2;

  if (currentState == BLINKING) {
    light1Yellow = (blinkingState) ? 4 : 2;
    light2Yellow = (blinkingState) ? 4 : 2;
  }

  lcd.setCursor(17, 1);
  lcd.write(light1Green);
  lcd.setCursor(17, 2);
  lcd.write(light1Yellow);
  lcd.setCursor(17, 3);
  lcd.write(light1Red);
  
  lcd.setCursor(19, 1);
  lcd.write(light2Green);
  lcd.setCursor(19, 2);
  lcd.write(light2Yellow);
  lcd.setCursor(19, 3);
  lcd.write(light2Red);

  // Display the presence sensors on the LCD
  byte sensor1 = is3l1.isVehiclePresent() ? 6 : 2;
  byte sensor2 = is2l1.isVehiclePresent() ? 6 : 2;
  byte sensor3 = is1l1.isVehiclePresent() ? 6 : 2;

  byte sensor4 = is3l2.isVehiclePresent() ? 6 : 2;
  byte sensor5 = is2l2.isVehiclePresent() ? 6 : 2;
  byte sensor6 = is1l2.isVehiclePresent() ? 6 : 2;

  lcd.setCursor(14, 1);
  lcd.write(sensor4);
  lcd.setCursor(14, 2);
  lcd.write(sensor5);
  lcd.setCursor(14, 3);
  lcd.write(sensor6);

  lcd.setCursor(15, 1);
  lcd.write(sensor1);
  lcd.setCursor(15, 2);
  lcd.write(sensor2);
  lcd.setCursor(15, 3);
  lcd.write(sensor3);
}

void updateTimingWeights(unsigned long currentMillis) {
  int vehicleCount1 = light1.getVehicleCount();
  int vehicleCount2 = light2.getVehicleCount();

  // Calculate the difference
  int diff = abs(vehicleCount1 - vehicleCount2);

  // Default green times
  unsigned long greenTime1 = GREEN_TIME1;
  unsigned long greenTime2 = GREEN_TIME2;

  if (vehicleCount1 == 0 && vehicleCount2 > 0) {
    greenTime2 = GREEN_TIME2 * DIFF3_MULT; 
    greenTime1 = GREEN_TIME2 * 2 - DIFF3_MULT; 
  } else if (vehicleCount2 == 0 && vehicleCount1 > 0) {
    greenTime2 = GREEN_TIME2 * 2 - DIFF3_MULT;
    greenTime1 = GREEN_TIME2 * DIFF3_MULT; 
  } else if (diff == 1) {
    if (vehicleCount1 > vehicleCount2) {
      greenTime1 = GREEN_TIME1 * DIFF1_MULT;
      greenTime2 = GREEN_TIME2 * 2.0 - DIFF1_MULT;
    } else {
      greenTime1 = GREEN_TIME1 * 2.0 - DIFF1_MULT;
      greenTime2 = GREEN_TIME2 * DIFF1_MULT;
    }
  } else if (diff >= 2) {
    if (vehicleCount1 > vehicleCount2) {
      greenTime1 = GREEN_TIME1 * DIFF2_MULT;
      greenTime2 = GREEN_TIME2 * 2 - DIFF2_MULT;
    } else {
      greenTime1 = GREEN_TIME1 * 2 - DIFF2_MULT;
      greenTime2 = GREEN_TIME2 * DIFF2_MULT;
    }
  }

  // Update the state duration based on the current state
  if (currentState == GREEN1_RED2) {
    stateDuration = greenTime1;
  } else if (currentState == RED1_GREEN2) {
    stateDuration = greenTime2;
  }
}

void updatePedestrianButtons(unsigned long currentMillis) {
  // Update button states
  button1.update();
  button2.update();

  if (pedestrianDebounce && (currentMillis - previousMillis >= PEDESTRIAN_DEBOUNCE)) pedestrianDebounce = false;
  // Handle button presses to transition faster
  if (!pedestrianDebounce && (button1.wasPressed() || button2.wasPressed())) {

    if (currentState == GREEN1_RED2 || currentState == RED1_GREEN2)
      stateDuration -= PEDESTRIAN_REDUCTION;

    button1.reset();
    button2.reset();
    pedestrianDebounce = true;
  }
}

void updateLightSensors(unsigned long currentMillis) {
  // Check for night mode
  bool nightMode = light1.isNight() || light2.isNight();

  if (nightMode && currentState != BLINKING) {
    // Switch to blinking state immediately
    light1.setBlinking();
    light2.setBlinking();
    currentState = BLINKING;
    previousMillis = currentMillis;
    stateDuration = BLINKING_DURATION;
  } else if (!nightMode && currentState == BLINKING) {
    // Switch back to initial state
    currentState = GREEN1_RED2;
    previousMillis = currentMillis;
    stateDuration = GREEN_TIME1;
    light1.setGreen();
    light2.setRed();
  }
}

void updateTrafficLights(unsigned long currentMillis) {
  // Update the state machine
  if (currentMillis - previousMillis >= stateDuration) {
    previousMillis = currentMillis;

    switch (currentState) {
      case GREEN1_RED2:
        light1.setYellow();
        light2.setRed();
        currentState = YELLOW1_RED2;
        stateDuration = YELLOW_TIME1;
        break;

      case YELLOW1_RED2:
        light1.setRed();
        light2.setGreen();
        currentState = RED1_GREEN2;
        stateDuration = GREEN_TIME2;
        break;

      case RED1_GREEN2:
        light1.setRed();
        light2.setYellow();
        currentState = RED1_YELLOW2;
        stateDuration = YELLOW_TIME2;
        break;

      case RED1_YELLOW2:
        light1.setGreen();
        light2.setRed();
        currentState = GREEN1_RED2;
        stateDuration = GREEN_TIME1;
        break;

      case BLINKING:
        blinkingState = !blinkingState;
        digitalWrite(LY1, blinkingState ? HIGH : LOW);
        digitalWrite(LY2, blinkingState ? HIGH : LOW);
        stateDuration = BLINKING_DURATION;
        break;
    }
  }
}

void sendJSON(unsigned long currentMillis) {
  if (currentMillis - previousSerialMillis < SERIAL_UPDATE_INTERVAL) return;

  previousSerialMillis = currentMillis;

  if (currentState != lastState) {  // Check if the state has changed
    lastState = currentState;       // Update the last state

    StaticJsonDocument<2048> doc;
    JsonObject data = doc.createNestedObject("data");
    data["currentState"] = (currentState == GREEN1_RED2) ? "GREEN1RED2" : (currentState == YELLOW1_RED2) ? "YELLOW1RED2"
                                                                       : (currentState == RED1_GREEN2)  ? "RED1GREEN2"
                                                                       : (currentState == RED1_YELLOW2) ? "RED1YELLOW2"
                                                                                                        : "BLINKING";
    data["stateDuration"] = stateDuration;

    JsonArray trafficLights = data.createNestedArray("trafficLights");

    JsonObject light1Obj = trafficLights.createNestedObject();
    light1Obj["trafficLightId"] = 1;
    light1Obj["totalOfVehicles"] = light1.getVehicleCount();
    light1Obj["isDaytime"] = !light1.isNight();
    light1Obj["co2Level"] = co2Sensor.GetValue();

    JsonObject light2Obj = trafficLights.createNestedObject();
    light2Obj["trafficLightId"] = 2;
    light2Obj["totalOfVehicles"] = light2.getVehicleCount();
    light2Obj["isDaytime"] = !light2.isNight();
    light2Obj["co2Level"] = 0;  // Assuming CO2 sensor is only for light1

    JsonObject states = doc.createNestedObject("state");
    states["GREEN_TIME1"] = GREEN_TIME1;
    states["YELLOW_TIME1"] = YELLOW_TIME1;
    states["RED_TIME1"] = RED_TIME1;
    states["GREEN_TIME2"] = GREEN_TIME2;
    states["YELLOW_TIME2"] = YELLOW_TIME2;
    states["RED_TIME2"] = RED_TIME2;
    states["BLINKING_DURATION"] = BLINKING_DURATION;

    String output;
    serializeJson(doc, output);
    Serial.println(output);
  }
}

void updateTimingFromGenAI() {
  if (!Serial.available()) return;
  String timingS = Serial.readString();
  StaticJsonDocument<256> response;
  deserializeJson(response, timingS);

  GREEN_TIME1 = (int) response["states"]["GREEN_TIME1"];
  YELLOW_TIME1 = (int) response["states"]["YELLOW_TIME1"];
  RED_TIME1 = (int) response["states"]["RED_TIME1"];
  GREEN_TIME2 = (int) response["states"]["GREEN_TIME2"];
  YELLOW_TIME2 = (int) response["states"]["YELLOW_TIME2"];
  RED_TIME2 = (int) response["states"]["RED_TIME2"];
  BLINKING_DURATION = (int) response["states"]["BLINKING_DURATION"];

  PEDESTRIAN_REDUCTION = (int) response["pedestrianReduction"];

  DIFF1_MULT = (float) response["vehicleCount"]["diff1"];
  DIFF2_MULT = (float) response["vehicleCount"]["diff2"];
  DIFF3_MULT = (float) response["vehicleCount"]["diff3"];
}

void setup() {
  // Initialize the LCD
  lcd.begin();
  lcd.backlight();

  // Create custom characters
  lcd.createChar(0, sunIcon);
  lcd.createChar(1, moonIcon);
  lcd.createChar(2, boxEmpty);
  lcd.createChar(3, boxRed);
  lcd.createChar(4, boxYellow);
  lcd.createChar(5, boxGreen);
  lcd.createChar(6, boxCarDetected);

  // Display startup messages
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Smart city");
  lcd.setCursor(0, 1);
  lcd.print("starting...");
  delay(2000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Checking sensors");

  // Check CO2 Sensor
  lcd.setCursor(0, 1);
  lcd.print("CO2: ");
  if (co2Sensor.GetValue() >= 0) {
    lcd.print("OK");
  } else {
    lcd.print("Error");
  }
  delay(1000);

  // Check Light Sensors
  lcd.setCursor(0, 3);
  lcd.print("Light sens: ");
  if (ls1.read() >= 0 && ls2.read() >= 0) {
    lcd.print("OK");
  } else {
    lcd.print("Error");
  }
  delay(1000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Setup complete");
  delay(2000);
  
  lcd.clear();

  // Initialize the state machine
  currentState = GREEN1_RED2;
  lastState = currentState;  // Initialize the last state
  previousMillis = millis();
  stateDuration = GREEN_TIME1;  // Initial green time for light 1

  light1.initialize();
  light1.setGreen();
  light2.initialize();
  light2.setRed();

  updateLCD(millis());

  Serial.begin(9600);
}

void loop() {
  unsigned long currentMillis = millis();

  updateTimingWeights(currentMillis);

  updatePedestrianButtons(currentMillis);

  updateLightSensors(currentMillis);

  updateTrafficLights(currentMillis);

  updateLCD(currentMillis);

  sendJSON(currentMillis);

  updateTimingFromGenAI();
}
