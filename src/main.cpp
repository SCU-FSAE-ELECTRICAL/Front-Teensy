#include <SPI.h>
#include <FlexCAN_T4.h>

void sendRequest(uint8_t Register);
void receiveCallback(const CAN_message_t &msg);
void sendCommand(uint8_t Register, int data);

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
int value = 0;
CAN_message_t txMsg;
unsigned long lastSendTime = 0;

// a14 
// a1 for both pedal inputs

#define BRAKE   A0
#define PEDAL1  A14
#define PEDAL2  A1
#define START   A3

void setup() {
  can1.begin();
  can1.setBaudRate(500000);
  can1.setMaxMB(16);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(receiveCallback);
  Serial.println("CAN initialized.");

  sendRequest(0x4A); // Request for Controller Temperature
  sendRequest(0x49); // Request for Motor Temperature
  sendRequest(0x30); // Request for RPM
  sendRequest(0x40); // Request for Ready Status
  sendRequest(0xEB); // Battery Voltage

  pinMode(START, INPUT);
  pinMode(BRAKE, INPUT);
  pinMode(PEDAL1, INPUT);
  pinMode(PEDAL2, INPUT);
}

void loop() {

  int pedal1Raw = analogRead(PEDAL1); // A14
  int pedal2Raw = analogRead(PEDAL2); // A1

  int mapped1 = constrain(map(pedal1Raw, 500, 280, 0, 5000), 0, 5000);
  int mapped2 = constrain(map(pedal2Raw, 450, 260, 0, 5000), 0, 5000);

  uint16_t averagePedal = (mapped1 + mapped2) / 2;
  sendCommand(0x31, averagePedal);

  Serial.print("Pedal A14: "); Serial.print(pedal1Raw);
  Serial.print(" ("); Serial.print(mapped1); Serial.print("%)");
  Serial.print("  Pedal A1: "); Serial.print(pedal2Raw);
  Serial.print(" ("); Serial.print(mapped2); Serial.print("%)");

  // Handle receive interrupts
  can1.events(); 
}

// Send a request to the motor controller
void sendRequest(uint8_t Register) {
  CAN_message_t msg;
  msg.id = 0x201;
  msg.len = 3;
  msg.buf[0] = 0x3D; // Parameter transmission request
  msg.buf[1] = Register; // REGID for the desired parameter
  msg.buf[2] = 250; // 250 ms cyclic transmission 

  can1.write(msg);  
}

void receiveCallback(const CAN_message_t &msg) {
  if (msg.id == 0x181)
  {
    Serial.print("Received ID: 0x");
    Serial.print(msg.id, HEX);
    Serial.print("  DATA: ");
    for (uint8_t i = 0; i < msg.len; i++) {
      Serial.print(msg.buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
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

