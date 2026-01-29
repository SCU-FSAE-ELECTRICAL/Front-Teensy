#include "functions.h"

int state = 0;
bool bspdLatched = false;

int cmdrpm = 0;
int maxrpm = 5500;

static const int BRAKE_THRESH = 25;
static const float OVERLAP_THROTTLE_PCT = 5.0f;   // overlap trigger
static const float CLEAR_THROTTLE_PCT   = 2.0f;   // clear latch

static uint32_t lastSend = 0;

static float throttlePctFromRaw(int raw) {
  raw = constrain(raw, 0, 1023);
  return (raw / 1023.0f) * 100.0f;
}

void updateDriverIO() {
  int brakeRaw = analogRead(BRAKE);
  int pedalRaw = analogRead(PEDAL1);

  bool brakePressed = (brakeRaw > BRAKE_THRESH);
  float throttlePct = throttlePctFromRaw(pedalRaw);

  if (state == 0) {
    cmdrpm = 0;
    bspdLatched = false;

    if (brakePressed && digitalRead(START) == HIGH && throttlePct < 2.0f) {
      state = 1;
    }
  } else {
    if (brakePressed && digitalRead(START) == HIGH && throttlePct < 2.0f) {
      state = 0;
      cmdrpm = 0;
      bspdLatched = false;
    }
  }

  if (state == 1) {
    if (!bspdLatched && brakePressed && throttlePct > OVERLAP_THROTTLE_PCT) {
      bspdLatched = true;
    }

    if (bspdLatched && throttlePct < CLEAR_THROTTLE_PCT) {
      bspdLatched = false;
    }

    if (bspdLatched) {
      cmdrpm = 0;
    } else {
      int rpmCmd = map(pedalRaw, 0, 1023, 0, maxrpm);
      if (rpmCmd < 30) rpmCmd = 0;
      cmdrpm = (int)((rpmCmd * 32767.0f) / maxrpm);
    }
  } else {
    cmdrpm = 0;
  }

  uint32_t now = millis();
  if (now - lastSend >= 10) {
    sendCommand(0x31, (uint16_t)cmdrpm);
    lastSend = now;
  }

  float displayRpm = (cmdrpm / 32767.0f) * maxrpm;
  setRpmBar(displayRpm);
}
