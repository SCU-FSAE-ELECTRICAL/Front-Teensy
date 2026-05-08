#include <SPI.h>
#include <FlexCAN_T4.h>

void sendRequest(uint8_t Register);
void receiveCallback(const CAN_message_t &msg);
void sendCommand(uint8_t Register, int data);

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
int value = 0;
CAN_message_t txMsg;
unsigned long lastSendTime = 0;
int drive = 0;

unsigned long lastPedalTime = 0;
unsigned long speakerStartTime = 0;
unsigned long lastButtonTime = 0;

const unsigned long PEDAL_INTERVAL = 20;
const unsigned long SPEAKER_TIME = 1000;
const unsigned long DEBOUNCE_TIME = 20;  // use 150 ms for a real button

bool buttonLatched = false;
bool speakerOn = false;

// a14 
// a1 for both pedal inputs

#define BRAKE   A0
#define PEDAL1  A14
#define PEDAL2  A1
#define BUTTON   A3
#define SPEAKER A2
#define LIGHT A15

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

  pinMode(BRAKE, INPUT);
  pinMode(PEDAL1, INPUT);
  pinMode(PEDAL2, INPUT);
  pinMode(BUTTON, INPUT);
  pinMode(SPEAKER, OUTPUT);
  pinMode(LIGHT, OUTPUT);
  digitalWrite(SPEAKER, LOW);
}

void loop() {
  unsigned long currentTime = millis();

  bool buttonState = digitalRead(BUTTON);  // pulldown: LOW = not pressed, HIGH = pressed

  // Button press detected
  if (buttonState == HIGH && buttonLatched == false) {
    if (currentTime - lastButtonTime > DEBOUNCE_TIME) {
      lastButtonTime = currentTime;
      buttonLatched = true;

      drive = !drive;

      if (drive == 1) {
        digitalWrite(LIGHT, HIGH);
        // Entering drive
        digitalWrite(SPEAKER, HIGH);
        speakerStartTime = currentTime;
        speakerOn = true;
      } else {
        digitalWrite(LIGHT, LOW);
        // Leaving drive
        digitalWrite(SPEAKER, LOW);
        speakerOn = false;
        sendCommand(0x31, 0);
      }
    }
  }

  // Wait for button release before allowing another toggle
  if (buttonState == LOW) {
    buttonLatched = false;
  }

  // Turn speaker off after 1 second
  if (speakerOn && currentTime - speakerStartTime >= SPEAKER_TIME) {
    digitalWrite(SPEAKER, LOW);
    speakerOn = false;
  }

  // Call getPedals every 20 ms only in drive mode
  if (drive == 1 && currentTime - lastPedalTime >= PEDAL_INTERVAL) {
    lastPedalTime = currentTime;
    getPedals();
  }

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
    /*Serial.print("Received ID: 0x");
    Serial.print(msg.id, HEX);
    Serial.print("  DATA: ");
    for (uint8_t i = 0; i < msg.len; i++) {
      Serial.print(msg.buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();*/
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

void getPedals()
{
  int pedal1Raw = analogRead(PEDAL1); // A14
  int pedal2Raw = analogRead(PEDAL2); // A1

  int mapped1 = constrain(map(pedal1Raw, 410, 195, 0, 5000), 0, 5000);
  int mapped2 = constrain(map(pedal2Raw, 358, 165, 0, 5000), 0, 5000);

  uint16_t averagePedal = (mapped1 + mapped2) / 2;
  Serial.println(averagePedal);
  if (averagePedal < 150)
    sendCommand(0x31, 0);
  else
    sendCommand(0x31, averagePedal);

  /*Serial.print("Pedal A14: "); Serial.print(pedal1Raw);
  Serial.print(" ("); Serial.print(mapped1); Serial.print("%)");
  Serial.print("  Pedal A1: "); Serial.print(pedal2Raw);
  Serial.print(" ("); Serial.print(mapped2); Serial.print("%)");*/
}
