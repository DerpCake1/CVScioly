#include <ezButton.h>         // ezButton library

#define MODE_BUTTON 12
#define START_BUTTON 13

// buttons
ezButton mode_button(MODE_BUTTON);
ezButton start_button(START_BUTTON);

void initButtons() {
  modde_button.setDebounceTime(50);
  start_button.setDebounceTime(50);
}

void handleButtons() {
  while (true) {
    mode_button.loop();
    start_button.loop();
    if(mode_button.isPressed())
      Serial.println("The mode button is pressed");
    if(start_button.isPressed())
      Serial.println("The start button is pressed");
    if(mode_button.isReleased())
      Serial.println("The mode button is released");
    if(start_button.isReleased())
      Serial.println("The mode button is released");
  }
}

void setup() {
  Serial.begin(115200);
  initButtons();
  Serial.println("Ready, press a button.");
  handleButtons();
}

void loop() {
}
