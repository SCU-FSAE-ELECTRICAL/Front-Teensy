#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <FastLED.h>

#define BRAKE   A0
#define PEDAL1  A14
#define START   A3

#define NUM_LEDS 10
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define BRIGHTNESS 80

extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;


extern int state;              // 0 = not enabled, 1 = enabled
extern bool bspdLatched;
extern int cmdrpm;             
extern int maxrpm;             

void initCAN();
void sendCommand(uint8_t reg, uint16_t data);

void initLEDs();
void setRpmBar(float rpm);
void setSocBar(float soc);

void updateDriverIO();

//imu functions
void initIMU();
bool readIMU();

float imuAx();
float imuAy();
float imuAz();

float imuYaw();
float imuPitch();
float imuRoll();



#endif
