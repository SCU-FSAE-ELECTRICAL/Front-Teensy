#include "functions.h"

/*
 * Power & RPM
 * Motor Temp
 * MT Temp
 * Cooland Temp/Pressure
 * Battery Voltage
 * High and Low Cell Voltage
 * High and Low acc temp
 * CAN Bus Monitoring/faults
 * SD Card Fault & Ejection
 * Non critical fault clear button
 */

int state = 0;
int ts_state = 0;
int IMD_state = 0;
int BMS_state = 0;

void setup() 
{
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  Serial2.begin(19200);
  //while(!Serial); // Nothing will happen until you open the serial monitor(for testing only)
  Serial.println("Initializing");
  initDriverInputs();
  initMPU();
  initSD();
  initCAN();
}

void plausibilityError()
{
  state = 0;
  cmdrpm = 0;
  sendCommand(0x31, cmdrpm);
  RTDSpeaker.start(250, 4, millis());
}

void loop() 
{
  unsigned long current_time = millis();

  getAccelerometerData();
  readMsg();
  getDriverInputs(current_time);
  updateRaspi();
}
