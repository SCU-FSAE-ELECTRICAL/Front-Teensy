#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <FastLED.h>

#define BRAKE   A0
#define PEDAL1  A14
#define START   A3

#define LEDPIN  6
#define NUM_LEDS 10
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define BRIGHTNESS 80

extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

extern CRGB leds[NUM_LEDS];

extern int state;              // 0 = not enabled, 1 = enabled
extern bool bspdLatched;
extern int cmdrpm;             // 0..32767 scaled
extern int maxrpm;             // 0..5500 rpm range (display + scaling)

void initCAN();
void sendCommand(uint8_t reg, uint16_t data);

void initLEDs();
void setRpmBar(float rpm);
void setSocBar(float soc);

void updateDriverIO();

#endif
