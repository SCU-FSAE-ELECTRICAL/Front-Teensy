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

const int potPin = A4;
int potValue;
float fakeRpm;


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

  pinMode(potPin, INPUT);
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
  getAccelerometerData();  // SD logging
  updateRaspi();           // Display updates
  checkSD();               // SD card health

  
  potValue = analogRead(potPin);
  fakeRpm = map(pot, 0 ,1023, 0, 5500);
  readMsg();          
  setRpmBar(fakeRpm);   
  Serial.println(potValue);  
  delay(10);       
  
}
