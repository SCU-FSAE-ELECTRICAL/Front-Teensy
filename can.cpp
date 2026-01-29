#include "functions.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

void initCAN() {
  can1.begin();
  can1.setBaudRate(500000);
}

void sendCommand(uint8_t reg, uint16_t data) {
  CAN_message_t msg;
  msg.id = 0x201;
  msg.len = 3;
  msg.buf[0] = reg;
  msg.buf[1] = data & 0xFF;
  msg.buf[2] = (data >> 8) & 0xFF;
  can1.write(msg);
}
