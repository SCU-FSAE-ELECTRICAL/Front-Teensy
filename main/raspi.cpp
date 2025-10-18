#include "functions.h"

unsigned long last_message_time = 0;
int raspi_connected = 0;

void updateRaspi()
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
    if (millis() - last_message_time > 0)
    {
      raspi_connected = 0;
      return;
    }

    Serial2.print("mtr_s=");
    Serial2.println(rpm);
    
    Serial2.print("pwr=");
    Serial2.println("-1");
    
    Serial2.print("acc_v=");
    Serial2.println("-1");
    
    Serial2.print("min_v=");
    Serial2.println("-1");

    Serial2.print("max_v=");
    Serial2.println("-1");

    Serial2.print("acc_t=");
    Serial2.println("-1");

    Serial2.print("mtr_t=");
    Serial2.println(Mtemperature);

    Serial2.print("cnt_t=");
    Serial2.println(Ctemperature);

    Serial2.print("cool_t=");
    Serial2.println("-1");
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
