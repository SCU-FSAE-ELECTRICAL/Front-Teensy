#pragma once

void initTorqueInterrupt();
void initFaultCheckInterrupt();
void initTempCheckInterrupt();
void torqueISR();
void faultCheckISR();
void tempCheckISR();
void resetFaults();  