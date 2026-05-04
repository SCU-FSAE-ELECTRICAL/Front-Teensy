#include "functions.h"
#include "temps.h"

#if SIMULATION_MODE
// simulated CAN values
int sim_bms = 0;
int sim_imd = 0;
int sim_bspd = 0;
int sim_rpm = 1500;
int sim_controller_temp = 45;
int sim_motor_temp = 40;
unsigned long sim_last_update = 0;
#endif

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

unsigned long lastInfoTime = 0;  // Track the last message time
unsigned long lastIMDTime = 0;
unsigned long lastBMSTime = 0;
unsigned long lastBSPDTime = 0;
const unsigned long TIMEOUT_PERIOD = 8000;  // Timeout period in milliseconds (8 seconds)


float Mres = 0; //Resistance in the thermistor
float Ctemperature = 0;
float Mtemperature = 0;
float rpm = 0;

void initCAN() {
  can1.begin();
  can2.begin();
  can1.setBaudRate(500000);
  can2.setBaudRate(500000);
  Serial.println("CAN initialized.");

  sendRequest(0x4A); // Request for Controller Temperature
  sendRequest(0x49); // Request for Motor Temperature
  sendRequest(0x30); // Request for RPM
  sendRequest(0x40); // Request for Ready Status
  sendRequest(0xEB); // Battery Voltage
}

void sendRequest(uint8_t Register) {
  CAN_message_t msg;
  msg.id = 0x201;
  msg.len = 3;
  msg.buf[0] = 0x3D; // Parameter transmission request
  msg.buf[1] = Register; // REGID for the desired parameter
  msg.buf[2] = 0xA; // 10ms cyclic transmission (can be 0-254)

  can1.write(msg);  
}

void sendCommand(uint8_t Register, int data) {
  // Set up the CAN message
  CAN_message_t msg;
  msg.id = 0x201;
  msg.len = 3;   // Message length in bytes
  msg.buf[0] = Register; // REGID for the desired parameter
  msg.buf[1] = data & 0xFF; // Byte 1 of data
  msg.buf[2] = data >> 8; // Byte 2 of data

  // Send the CAN message
  can1.write(msg);
}

// -------------------------------------------------------------------------- //
void sendInfo(int id, int len, uint8_t var, int data) {
  // Set up the CAN message
  CAN_message_t msg;
  msg.id = id & 0x8FF; // 11-bit identifier
  msg.len = len;   // Message length in bytes
  msg.buf[0] = var; // Variable for the desired parameter
  for (int i = 1; i < len; i++) { // Sends individual bytes of data
    msg.buf[i] = data >> (8 * (i - 1)); // Calculates offset, LSB first
  }

  // Send the CAN message
  can1.write(msg);
}

// Handles CAN timeout errors.
void handleError() {
  Serial.println("CAN timeout error occurred!");
}

// Handles plasability error
void plausabilityError() {
  Serial.println("Pedal Error");
  digitalWrite(LED_BUILTIN, HIGH);
  // Stops all power to the motor, not implementing until we get a good jig and testing - not restartable unless through NDrive
  //sendCommand(0x51, 0x0004);
}

void readMsg() {
  #if SIMULATION_MODE
    // ------------ SIMULATION MODE ------------
    static unsigned long sim_last = 0;
    if (millis() - sim_last > 10) {  // simulate 100 Hz CAN input
      sim_last = millis();

      // Fake temperatures rising slowly
      Ctemperature += 0.02;
      Mtemperature += 0.015;

      // Fake RPM waveform
      rpm = 2000 + 200 * sin(millis() / 200.0);

      // Fake BMS/IMD/BSPD toggling
      //fault_BMS  = ((millis() / 5000) % 2);
      //fault_IMD  = ((millis() / 7000) % 2);
      //fault_BSPD = ((millis() / 9000) % 2);

      // Fake heartbeat timeout flag
      fault_rear_teensy = ((millis() % 8000) < 500);

      // Update last timestamps so the timeout logic mirrors real behavior
      lastBMSTime  = millis();
      lastIMDTime  = millis();
      lastBSPDTime = millis();

      // Optional debugging
      Serial.print("SIM CAN | RPM=");
      Serial.print(rpm);
      Serial.print("  Ctemp=");
      Serial.print(Ctemperature);
      Serial.print("  Mtemp=");
      Serial.print(Mtemperature);
      Serial.print("  BMS=");
      Serial.print(fault_BMS);
      Serial.print(" IMD=");
      Serial.print(fault_IMD);
      Serial.print(" BSPD=");
      Serial.println(fault_BSPD);
    }

    return; // ⚠ IMPORTANT: Skip all real CAN when simulating
  #endif

  CAN_message_t msg;
  // Check if there are any messages available on the CAN bus
  while (can1.read(msg)) {
    //Print CAN message details for debugging
    /*Serial.print("CAN1 ");
    Serial.print("  ID: 0x");
    Serial.print(msg.id, HEX);
    Serial.print("  EXT: ");
    Serial.print(msg.flags.extended);
    Serial.print("  LEN: ");
    Serial.print(msg.len);
    Serial.print(" DATA: ");
    for (uint8_t i = 0; i < msg.len; i++) {
       Serial.print(msg.buf[i], HEX);
       Serial.print(" ");
    }
    Serial.println("");
    Serial.println();
    Serial.print("  TS: ");
    Serial.println(msg.timestamp);*/
    
    // Motor Controller
    if (msg.id == 0x181) {
      //lastMessageTime = millis();
      uint32_t value = (msg.buf[2] << 8) | msg.buf[1];

      // Process Controller Temperature
      if (msg.buf[0] == 0x4A) {
        Ctemperature = getControllerTemp((float)value);
        logToSD(0x181, msg.buf[0], Ctemperature);
        //Serial.print("Controller Temperature: ");
        //Serial.println(Ctemperature); // Print the temperature value
      }
      // Process Motor Temperature
      else if (msg.buf[0] == 0x49) {
        Mres = ((float)value * 4000) / (32000 - value);
        Mtemperature = getMotorTemp(value);
        logToSD(0x181, msg.buf[0], Mtemperature);
        //Serial.print("Motor Temperature: ");
        //Serial.println(Mtemperature); // Print the temperature value
      }
      // Process RPM
      else if (msg.buf[0] == 0x30) {
        rpm = ((float)value / 32767) * 5500;
        logToSD(0x181, msg.buf[0], rpm);
        //Serial.print("RPM: ");
        //Serial.println(rpm); // Print the RPM value
      }
      // Process Ready Status
      else if (msg.buf[0] == 0x40) {
        //Serial.println(msg.buf[1] >> 6 & 0x01);
      }
      else if (msg.buf[0] == 0xEB) {
        //uint16_t raw_voltage = (msg[3] << 8) | msg[2];
        //float voltage = raw_voltage * 0.1;  
        //Serial.println(voltage);
      }
    }
  }
  while (can2.read(msg)) // write out placeholders for message from BMS(voltages + temps)
  {
    // ----- BMS -----
    if (msg.id == 0x105 && msg.buf[0] == 0x77) 
    {
      fault_BMS = (msg.buf[1] != 0);
      lastBMSTime = millis();
    }

    // ----- IMD -----
    else if (msg.id == 0x106 && msg.buf[0] == 0x66) 
    {
      fault_IMD = (msg.buf[1] != 0);
      lastIMDTime = millis();
    }

    // ----- BSPD -----
    else if (msg.id == 0x107 && msg.buf[0] == 0x55) 
    {
      fault_BSPD = (msg.buf[1] != 0);
      lastBSPDTime = millis();
    }
  }
  // ---------- HEARTBEAT TIMEOUT ----------
  unsigned long now = millis();

  // Rear Teensy stopped transmitting then FAULT
  if (now - lastBMSTime > 500 || now - lastIMDTime > 500) 
  {
    fault_rear_teensy = true;
  } else {
    fault_rear_teensy = false;
  }
}



