#ifndef BUTTON_H
#define BUTTON_H

class Button {
private:
  int pin;
  bool wasPressedInCycle;
  unsigned long lastDebounceTime;
  unsigned long debounceDelay;

public:
  Button(int pin, unsigned long debounceDelay = 50);
  void update();
  bool wasPressed();
  void reset();
};

#endif
