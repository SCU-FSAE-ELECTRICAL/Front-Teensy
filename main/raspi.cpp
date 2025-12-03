#include "functions.h"

unsigned long last_message_time = 0;
int raspi_connected = 0;

void updateRaspi() // make sure all messages correlate to what is seen on raspi
{
  if (raspi_connected == 0)
  {
    waitForSerial();
  }
  else
  {
    if (Serial2.available() > 0)
    {
      String recived = Serial2.readStringUntil('\n');
      if (recived == "check")
      {
        last_message_time = millis();
      }
    }
    if (millis() - last_message_time > 5000)  // 5 second timeout
    {
      raspi_connected = 0;
      return;
    }

    // Motor speed
    Serial2.print("mtr_s=");
    Serial2.println(rpm);
    
    // Power (calculate from RPM - this is approximate)
    float power = (rpm / 5500.0) * 800.0;  // Scale to ~800W max
    Serial2.print("pwr=");
    Serial2.println(power);
    
    // Accumulator voltage 
    Serial2.print("acc_v=");
    Serial2.println("15.5");
    
    // Min cell voltage 
    Serial2.print("min_v=");
    Serial2.println("3.200");

    // Max cell voltage
    Serial2.print("max_v=");
    Serial2.println("4.100");

    // Accumulator temperature (Avg & High)
    Serial2.print("acc_t=");
    Serial2.println("35.0");

    // Motor temperature
    Serial2.print("mtr_t=");
    Serial2.println(Mtemperature);

    // Controller temperature
    Serial2.print("cnt_t=");
    Serial2.println(Ctemperature);

    // Coolant temperature 
    Serial2.print("cool_t=");
    Serial2.println("30.0");
    
    // System state (0 = idle, 1 = enabled/driving)
    Serial2.print("status=");
    Serial2.println(state);
    
    // Tractive system active (1 when SDC is closed)
    Serial2.print("ts_active=");
    Serial2.println(state);  // Use same as status for now
    
    // SD card status
    Serial2.print("sd=");
    Serial2.println(SD_state);
    
    // Brake status
    Serial2.print("brk=");
    Serial2.println(brake ? 1 : 0);
    
    // Gas/accelerator status (>5% pedal position)
    int apps_avg = (analogRead(PEDAL1) + analogRead(PEDAL2)) / 2;
    int gas_active = (apps_avg > 51) ? 1 : 0;  // 5% threshold
    Serial2.print("gas=");
    Serial2.println(gas_active);
    
    // Faults
    Serial2.print("fault_imd=");
    Serial2.println(fault_IMD ? 1 : 0);

    Serial2.print("fault_bms=");
    Serial2.println(fault_BMS ? 1 : 0);

    Serial2.print("fault_bspd=");
    Serial2.println(fault_BSPD ? 1 : 0);

    Serial2.print("fault_rear_teensy=");
    Serial2.println(fault_rear_teensy ? 1 : 0);
  }
}

void waitForSerial()
{
  if (Serial2.available() > 0)
  {
    String receivedString = Serial2.readStringUntil('\n');
    if (receivedString == "pi_ready")
    {
      Serial2.println("rodger");
      last_message_time = millis();
      raspi_connected = 1;
    }
  }
}