#ifndef FUNCTIONS
#define FUNCTIONS

#include <Arduino.h>
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <FlexCAN_T4.h>
#include <SD.h>
#include <SPI.h>

#define BRAKE A0
#define BRAKE2 A11
#define PEDAL1 A14
#define PEDAL2 A1
#define STEER A10

#define START A3
#define SPEAKER A2

#define IMD_LED A16
#define BMS_LED A15

class Buzzer {
  int pin;
  unsigned long pulseTime;
  int totalPulses;
  int currentPulse = 0;
  bool isOn = false;
  unsigned long lastToggleTime = 0;
  bool active = false;

public:
  Buzzer(int p) : pin(p) {}

  void start(unsigned long timePerState, int pulses, unsigned long currentTime) {
    pulseTime = timePerState;
    totalPulses = pulses;
    currentPulse = 0;
    isOn = false;
    active = true;
    lastToggleTime = currentTime;
    digitalWrite(pin, LOW);
  }

  void update() {
    if (!active) return;

    if (millis() - lastToggleTime >= pulseTime) {
      lastToggleTime = millis();

      isOn = !isOn;
      digitalWrite(pin, isOn ? HIGH : LOW);

      if (!isOn) {
        currentPulse++;
        if (currentPulse >= totalPulses) {
          active = false;
        }
      }
    }
  }

  void stop() {
    active = false;
    isOn = false;
    digitalWrite(pin, LOW);
  }
};

extern int state;
extern int ts_state;
extern int can1_state;
extern int can2_state;
extern int SD_state;
extern int BMS_state;
extern int IMD_state;
extern float battery_level;
extern float low_temp;
extern float high_temp;
extern unsigned long lastCommandTime;
extern unsigned long sensorDiffTime;
extern float Ctemperature;
extern float Mtemperature;
extern float rpm;
extern int cmdrpm;
extern Buzzer RTDSpeaker;

void initMPU(void);
void calibrateMPU(void);
void initSD(void);
void initSmallLCD(void);
void initCAN(void);
void initDriverInputs();
void getAccelerometerData(void);
void logToSD(uint8_t canID, uint8_t deviceID, String data);
void closeSD();
void checkSD();
void getDriverInputs(unsigned long current_time);
void getThrottle(unsigned long current_time);
void getBrake();
void disableDriverControl();
void plausibilityError();
void sendRequest(uint8_t Register);
void sendCommand(uint8_t Register, int data);
void readMsg();
void updateRaspi();
void waitForSerial();

#endif
