#include <Arduino.h>
#include <IntervalTimer.h>
#include "functions.h"
#include "temps.h"

IntervalTimer torqueTimer;
IntervalTimer faultCheckTimer;
IntervalTimer tempCheckTimer;

// Fault state variables
//static unsigned long appsFaultStart = 0;
static bool appsFaultLatched = false;
static bool btoFaultLatched = false;
static bool brakeFaultLatched = false;
static bool appsVoltageFaultLatched = false;
static bool brakeVoltageFaultLatched = false;
static bool overTempFaultLatched = false;

// Last known good brake values for plausibility
//static int lastBrake1 = 0;
//static int lastBrake2 = 0;


//static unsigned long brakeFaultStart = 0;

// Voltage thresholds (assuming 10-bit ADC, 0-5V range)
// 0.5V = 102, 4.5V = 921 (out of 1023)
const int VOLTAGE_MIN = 102;
const int VOLTAGE_MAX = 921;

const int BRAKE_THRESHOLD = 25;

//can IDs (need to be checked)
const uint16_t CAN_BRAKE_LIGHT_ID = 0x108;
const uint8_t BRAKE_LIGHT_DATA_ID = 0x42;

// ============================================================================
// TORQUE CONTROL INTERRUPT - 100 Hz (10ms)
// ============================================================================
void torqueISR() {
    int pedal1 = analogRead(PEDAL1);
    int pedal2 = analogRead(PEDAL2);

    int pedal1Calibrated = constrain(pedal1, 282, 520);
    int pedal2Calibrated = constrain(pedal2, 255, 461);

    float pedal1_pct = ((float)(520 - pedal1Calibrated) / (520 - 282)) * 100.0;
    float pedal2_pct = ((float)(461 - pedal2Calibrated) / (461 - 255)) * 100.0;

    float apps_pct = (pedal1_pct + pedal2_pct) / 2.0;

    int rpm_cmd = (int)((apps_pct / 100.0) * 5500);

    if (rpm_cmd < 30) rpm_cmd = 0;

    cmdrpm = rpm_cmd * 32767 / 5500;

    sendCommand(0x31, cmdrpm);

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200) {
        Serial.print("APPS1=");
        Serial.print(pedal1);
        Serial.print(" APPS2=");
        Serial.print(pedal2);
        Serial.print(" APPS_PCT=");
        Serial.print(apps_pct, 1);
        Serial.print("% | rpm_cmd=");
        Serial.print(rpm_cmd);
        Serial.print(" | cmdrpm=");
        Serial.println(cmdrpm);
        lastPrint = millis();
    }
}


/*
void torqueISR() {
    #if SIMULATION_MODE
    static unsigned long sim_last = 0;
    static float sim_apps = 0;  
    
    if (millis() - sim_last > 10) {
      sim_last = millis();

      sim_apps += 0.02;
      if (sim_apps > 100) sim_apps = 0;

      Ctemperature += 0.02;
      Mtemperature += 0.015;

      rpm = 2000 + 200 * sin(millis() / 200.0);

      fault_BMS  = ((millis() / 5000) % 2);
      fault_IMD  = ((millis() / 7000) % 2);
      fault_BSPD = ((millis() / 9000) % 2);

      fault_rear_teensy = ((millis() % 8000) < 500);

      lastBMSTime  = millis();
      lastIMDTime  = millis();
      lastBSPDTime = millis();
    }
    
    brake = (millis() / 1500) % 2;
    
    if (brake && sim_apps > 25) {
        fault_state = true;
    }

    cmdrpm = fault_state ? 0 : (sim_apps * 55);

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
        Serial.print("APPS="); Serial.print(sim_apps, 1);
        Serial.print("% | Brake="); Serial.print(brake ? "ON " : "OFF");
        Serial.print(" | RPM="); Serial.print(cmdrpm);
        Serial.print(" | Faults: ");
        if (fault_BMS) Serial.print("BMS ");
        if (fault_IMD) Serial.print("IMD ");
        if (fault_BSPD) Serial.print("BSPD ");
        if (fault_rear_teensy) Serial.print("REAR_TEENSY ");
        if (fault_state) Serial.print("[TORQUE=0]");
        Serial.println();
        lastPrint = millis();
    }

    return;
    #endif

    int pedal1 = analogRead(PEDAL1);
    int pedal2 = analogRead(PEDAL2);
    int brake1_raw = analogRead(BRAKE);
    int brake2_raw = analogRead(BRAKE2);

    // APPS voltage range check
    if (pedal1 < VOLTAGE_MIN || pedal1 > VOLTAGE_MAX ||
        pedal2 < VOLTAGE_MIN || pedal2 > VOLTAGE_MAX) {
        appsVoltageFaultLatched = true;
    }

    // Brake voltage range check
    if (brake1_raw < VOLTAGE_MIN || brake1_raw > VOLTAGE_MAX ||
        brake2_raw < VOLTAGE_MIN || brake2_raw > VOLTAGE_MAX) {
        brakeVoltageFaultLatched = true;
    }

    // Brake plausibility check
    int brake_diff = abs(brake1_raw - brake2_raw);
    if (brake_diff > 102) {
        if (brakeFaultStart == 0) brakeFaultStart = millis();
        if (millis() - brakeFaultStart > 100) {
            brakeFaultLatched = true;
        }
    } else {
        brakeFaultStart = 0;
    }

    bool brake_pressed = ((brake1_raw + brake2_raw) / 2) > BRAKE_THRESHOLD; 
    brake = brake_pressed;

    float pedal1_pct = (pedal1 / 1023.0) * 100.0;
    float pedal2_pct = (pedal2 / 1023.0) * 100.0;
    
    float diff_pct = abs(pedal1_pct - pedal2_pct);

    if (diff_pct > 10.0) {
        if (appsFaultStart == 0) appsFaultStart = millis();
        if (millis() - appsFaultStart > 100) {
            appsFaultLatched = true;
        }
    } else {
        appsFaultStart = 0;
    }

    bool throttleAbove25 = (pedal1_pct > 25.0 || pedal2_pct > 25.0);

    if (brake_pressed && throttleAbove25) {
        btoFaultLatched = true;
    }

    if (btoFaultLatched && pedal1_pct < 5.0 && pedal2_pct < 5.0) {
        btoFaultLatched = false;
    }

    if (appsFaultLatched || btoFaultLatched || brakeFaultLatched ||
        appsVoltageFaultLatched || brakeVoltageFaultLatched ||
        overTempFaultLatched ||
        fault_BMS || fault_IMD || fault_BSPD || fault_rear_teensy) {
        
        cmdrpm = 0;
        sendCommand(0x31, 0);
        fault_state = true;
        
        sendInfo(CAN_BRAKE_LIGHT_ID, 2, BRAKE_LIGHT_DATA_ID, brake_pressed ? 1 : 0);

        static unsigned long lastPrintReal = 0;
        if (millis() - lastPrintReal > 500) {
            Serial.print("FAULT ACTIVE | RPM=0 | Faults: ");
            if (appsFaultLatched) Serial.print("APPS_MISMATCH ");
            if (btoFaultLatched) Serial.print("BTO ");
            if (brakeFaultLatched) Serial.print("BRAKE_PLAUS ");
            if (appsVoltageFaultLatched) Serial.print("APPS_VOLTAGE ");
            if (brakeVoltageFaultLatched) Serial.print("BRAKE_VOLTAGE ");
            if (overTempFaultLatched) Serial.print("OVERTEMP ");
            if (fault_BMS) Serial.print("BMS ");
            if (fault_IMD) Serial.print("IMD ");
            if (fault_BSPD) Serial.print("BSPD ");
            if (fault_rear_teensy) Serial.print("REAR_TEENSY ");
            Serial.println();
            lastPrintReal = millis();
        }
        
        return;
    }

    fault_state = false;
    
    int apps_avg = (pedal1 + pedal2) / 2;
    int rpm_cmd = map(apps_avg, 0, 1023, 0, 5500);

    if (rpm_cmd < 30) rpm_cmd = 0;
    if (brake_pressed) rpm_cmd = 0;

    cmdrpm = rpm_cmd * 32767 / 5500;
    sendCommand(0x31, cmdrpm);

    static bool lastBrakeState = false;
    if (brake_pressed != lastBrakeState) {
        sendInfo(CAN_BRAKE_LIGHT_ID, 2, BRAKE_LIGHT_DATA_ID, brake_pressed ? 1 : 0);
        lastBrakeState = brake_pressed;
    }
    
    static unsigned long lastPrintReal2 = 0;
    if (millis() - lastPrintReal2 > 200) {
        Serial.print("APPS1="); Serial.print(pedal1);
        Serial.print(" APPS2="); Serial.print(pedal2);
        Serial.print(" ("); Serial.print(pedal1_pct, 1); Serial.print("%, ");
        Serial.print(pedal2_pct, 1); Serial.print("%)");
        Serial.print(" | Brake="); Serial.print(brake_pressed ? "ON " : "OFF");
        Serial.print(" | RPM_CMD="); Serial.print(cmdrpm);
        Serial.println();
        lastPrintReal2 = millis();
    }
}
*/




// ============================================================================
// FAULT CHECK INTERRUPT - 100 Hz (10ms)
// Monitors BMS, IMD, BSPD, and heartbeat from rear MCU
// ============================================================================

void faultCheckISR() {
    fault_rear_teensy = false;
    fault_BMS = false;
    fault_IMD = false;
    fault_BSPD = false;
}

/*
void faultCheckISR() {
    CAN_message_t msg;
    
    // Read all fault messages from CAN2
    while (can2.read(msg)) {
        if (msg.id == 0x105 && msg.buf[0] == 0x77) {
            fault_BMS = (msg.buf[1] != 0);
            lastBMSTime = millis();
        }
        else if (msg.id == 0x106 && msg.buf[0] == 0x66) {
            fault_IMD = (msg.buf[1] != 0);
            lastIMDTime = millis();
        }
        else if (msg.id == 0x107 && msg.buf[0] == 0x55) {
            fault_BSPD = (msg.buf[1] != 0);
            lastBSPDTime = millis();
        }
    }
    
    // Heartbeat timeout check (500ms timeout)
    unsigned long now = millis();
    if (now - lastBMSTime > 500 || now - lastIMDTime > 500) {
        fault_rear_teensy = true;
    } else {
        fault_rear_teensy = false;
    }
}
*/

// ============================================================================
// TEMPERATURE CHECK INTERRUPT - 2 Hz (500ms)
// Monitors motor and controller temperatures
// ============================================================================
void tempCheckISR() {
    CAN_message_t msg;
    
    // Read temperature messages from motor controller (CAN1)
    while (can1.read(msg)) {
        if (msg.id == 0x181) {
            uint32_t value = (msg.buf[2] << 8) | msg.buf[1];
            
            // Controller temperature
            if (msg.buf[0] == 0x4A) {
                Ctemperature = getControllerTemp((float)value);
                
                // CRITICAL: Controller over 85°C → fault
                if (Ctemperature > 85.0) {
                    overTempFaultLatched = true;
                }
                
                logToSD(0x181, msg.buf[0], Ctemperature);
            }
            // Motor temperature
            else if (msg.buf[0] == 0x49) {
                Mtemperature = getMotorTemp(value);
                
                // CRITICAL: Motor over 120°C → fault
                if (Mtemperature > 120.0) {
                    overTempFaultLatched = true;
                }
                
                logToSD(0x181, msg.buf[0], Mtemperature);
            }
            // RPM (non-critical, just log)
            else if (msg.buf[0] == 0x30) {
                rpm = ((float)value / 32767) * 5500;
                logToSD(0x181, msg.buf[0], rpm);
            }
        }
    }
}

// ============================================================================
// INITIALIZATION FUNCTIONS
// ============================================================================
void initTorqueInterrupt() {
    torqueTimer.begin(torqueISR, 10000);  // 10ms = 100Hz
}

void initFaultCheckInterrupt() {
    faultCheckTimer.begin(faultCheckISR, 10000);  // 10ms = 100Hz
}

void initTempCheckInterrupt() {
    tempCheckTimer.begin(tempCheckISR, 500000);  // 500ms = 2Hz
}

// ============================================================================
// FAULT RESET FUNCTION (must be called manually, not automatic)
// ============================================================================
void resetFaults() {
    // Only allow reset if ALL conditions are safe:
    // 1. APPS < 5%
    // 2. Brake not pressed
    // 3. No active shutdown faults (BMS, IMD, BSPD)
    
    int pedal1 = analogRead(PEDAL1);
    int pedal2 = analogRead(PEDAL2);
    int brake_raw = analogRead(BRAKE);
    
    float pedal1_pct = (pedal1 / 1023.0) * 100.0;
    float pedal2_pct = (pedal2 / 1023.0) * 100.0;
    bool brake_pressed = (brake_raw > 25);
    
    // Only reset if safe conditions met
    if (pedal1_pct < 5.0 && pedal2_pct < 5.0 && !brake_pressed &&
        !fault_BMS && !fault_IMD && !fault_BSPD && !fault_rear_teensy) {
        
        appsFaultLatched = false;
        btoFaultLatched = false;
        brakeFaultLatched = false;
        appsVoltageFaultLatched = false;
        brakeVoltageFaultLatched = false;
        overTempFaultLatched = false;
        fault_state = false;
        
        Serial.println("FAULTS RESET - System ready");
    } else {
        Serial.println("CANNOT RESET - Unsafe conditions");
    }
}