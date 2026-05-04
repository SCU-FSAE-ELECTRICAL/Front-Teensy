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

const int rpmPotPin = A4;
const int socPotPin = A5;
int rpmPotValue;
int socPotValue;

/*
float fakeRpm;
float fakeSoc;
*/

bool fault_BMS = false;
bool fault_IMD = false;
bool fault_BSPD = false;
bool fault_rear_teensy = false;
bool fault_state = false;


void setup() 
{
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  Serial2.begin(19200);
  
  Serial.println("Initializing");
  initDriverInputs();
  initMPU();
  initSD();
  initCAN();

  //LED Setup
  initLEDs();
  
  //pinMode(START, INPUT_PULLUP); -enables internal pullup

  // Initialize ALL interrupts
  initTorqueInterrupt();      // 100 Hz - Torque control
  initFaultCheckInterrupt();  // 100 Hz - Fault monitoring
  initTempCheckInterrupt();   // 2 Hz - Temperature monitoring
  
  Serial.println("All interrupts initialized");

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
  getAccelerometerData();
  updateRaspi();
  checkSD();

  // readMsg();

  delay(10);
}
