#include "TrafficLight.h"
#include "Button.h"

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

#define GREEN_TIME1 3500       // Green time for light 1
#define YELLOW_TIME1 500       // Yellow time for light 1
#define RED_TIME1 4000         // Red time for light 1
#define GREEN_TIME2 3500       // Green time for light 2
#define YELLOW_TIME2 500       // Yellow time for light 2
#define RED_TIME2 4000         // Red time for light 2
#define PEDESTRIAN_DEBOUNCE 2000

#define DEBOUNCE_DELAY 500  // Debounce delay in milliseconds


Button button1(P2, DEBOUNCE_DELAY);
Button button2(P1, DEBOUNCE_DELAY);
TrafficLight light1(LR1, LY1, LG1, button1);  // Semáforo 1
TrafficLight light2(LR2, LY2, LG2, button2);  // Semáforo 2

enum TrafficLightState {
  GREEN1_RED2,
  YELLOW1_RED2,
  RED1_GREEN2,
  RED1_YELLOW2
};

TrafficLightState currentState;
unsigned long previousMillis = 0;
unsigned long stateDuration = 0;
bool pedestrianDebounce = false;

void setup() {
  // Inicializa la máquina de estados
  currentState = GREEN1_RED2;
  previousMillis = millis();
  stateDuration = GREEN_TIME1;  // Initial green time for light 1

  light1.initialize();
  light1.setGreen();
  light2.initialize();
  light2.setRed();
  Serial.begin(9600);
}

void loop() {
  unsigned long currentMillis = millis();

  // Update button states
  button1.update();
  button2.update();

  if (pedestrianDebounce && (currentMillis - previousMillis >= PEDESTRIAN_DEBOUNCE)) pedestrianDebounce = false;
  // Handle button presses to transition faster
  if (!pedestrianDebounce && (button1.wasPressed() || button2.wasPressed())) {

    if (currentState == GREEN1_RED2 || currentState == RED1_GREEN2)
      stateDuration = 0;  // Transition to yellow immediately

    Serial.println(currentState);
    Serial.println(stateDuration);
    button1.reset();
    button2.reset();
    pedestrianDebounce = true;
  }

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
    }
  }
}
