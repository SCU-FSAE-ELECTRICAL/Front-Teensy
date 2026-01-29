#include "functions.h"

void setup() {
  Serial.begin(115200);

  pinMode(START, INPUT);
  pinMode(BRAKE, INPUT);
  pinMode(PEDAL1, INPUT);

  initCAN();
  initLEDs();
}

void loop() {
  updateDriverIO();
}
