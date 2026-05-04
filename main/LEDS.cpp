#include <FastLED.h>
#include "functions.h"

#define NUM_LEDS 10
#define COLOR_ORDER GRB
#define LEDPIN 6
#define BRIGHTNESS 80
#define LED_TYPE WS2812B


//list of led colors
static CRGB leds[NUM_LEDS];

const float min_rpm = 0.0f;
const float max_rpm = 5500.0f;

void initLEDs()
{
  FastLED.addLeds<LED_TYPE, LEDPIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear(true);
}

CRGB colorForIndex (int index)
{
  if (index <= 2) return CRGB::Green;
  if (index <= 6) return CRGB::Yellow;
  return CRGB::Red;
}

int mapRpmToLED (float rpm)
{
  if (rpm <= min_rpm) return 0;
  if (rpm >= max_rpm) return NUM_LEDS;

  float rpmPerLED = (max_rpm - min_rpm) / NUM_LEDS;
  int lit = (int)ceil((rpm - min_rpm) / rpmPerLED);

  if (lit < 0) lit = 0;
  if (lit > NUM_LEDS) lit = NUM_LEDS;
  return lit;
}

void setRpmBar(float rpm)
{
  int lit = mapRpmToLED(rpm);
  
  for(int i = 0; i < NUM_LEDS; ++i)
  {
    if (i < lit) leds[i] = colorForIndex(i);
    else leds[i] = CRGB::Black;
  }

  FastLED.show();
}