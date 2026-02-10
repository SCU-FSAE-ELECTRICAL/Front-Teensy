#include <FastLED.h>
#include "functions.h"


#define RPM_LED_PIN 6
#define SOC_LED_PIN 7

static CRGB rpmLeds[NUM_LEDS];
static CRGB socLeds[NUM_LEDS];

const float min_rpm = 0.0f;
const float max_rpm = 5500.0f;

void initLEDs() {
  FastLED.addLeds<LED_TYPE, RPM_LED_PIN, COLOR_ORDER>(rpmLeds, NUM_LEDS);
  FastLED.addLeds<LED_TYPE, SOC_LED_PIN, COLOR_ORDER>(socLeds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear(true);
}

CRGB rpmColor(int i) {
  if (i <= 2) return CRGB::Green;
  if (i <= 6) return CRGB::Yellow;
  return CRGB::Red;
}

CRGB socColor(int i) {
  if (i <= 2) return CRGB::Red;
  if (i <= 6) return CRGB::Yellow;
  return CRGB::Green;
}

int mapRpm(float rpm) {
  if (rpm <= min_rpm) return 0;
  if (rpm >= max_rpm) return NUM_LEDS;

  float step = (max_rpm - min_rpm) / NUM_LEDS;
  int lit = (int)((rpm - min_rpm) / step) + 1;
  if (lit > NUM_LEDS) lit = NUM_LEDS;
  return lit;
}

int mapSoc(float soc) {
  if (soc <= 0.0f) return 0;
  if (soc >= 100.0f) return NUM_LEDS;

  int lit = (int)((soc / 100.0f) * NUM_LEDS) + 1;
  if (lit > NUM_LEDS) lit = NUM_LEDS;
  return lit;
}

void setRpmBar(float rpm) {
  int lit = mapRpm(rpm);
  for (int i = 0; i < NUM_LEDS; i++)
    rpmLeds[i] = (i < lit) ? rpmColor(i) : CRGB::Black;
  FastLED.show();
}

void setSocBar(float soc) {
  int lit = mapSoc(soc);
  for (int i = 0; i < NUM_LEDS; i++)
    socLeds[i] = (i < lit) ? socColor(i) : CRGB::Black;
  FastLED.show();
}