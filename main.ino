#include "functions.h"
#include <Wire.h>

void setup() {
  Serial.begin(115200);

  pinMode(START, INPUT);
  pinMode(BRAKE, INPUT);
  pinMode(PEDAL1, INPUT);

  initCAN();
  initLEDs();

  //IMU
  Wire.begin();
  Wire.setClock(400000);
  initIMU();
}

void loop() {
  updateDriverIO();

  //IMU
  static unsigned long lastIMU = 0;

  if (millis() - lastIMU >= 10) {  
    lastIMU = millis();

    readIMU();

    Serial.print("Yaw: "); Serial.print(imuYaw());
    Serial.print(" Pitch: "); Serial.print(imuPitch());
    Serial.print(" Roll: "); Serial.println(imuRoll());
  }
}
