#include "functions.h"

static File logFile;
int SD_state = 0;
int doSDLogging = 1;

void initSD() {
  if (!SD.begin(BUILTIN_SDCARD)) {
      Serial.println("SD Card initialization failed!");
      Serial2.println("sdc=0");
  }
  else
  {
    logFile = SD.open("log.csv", FILE_WRITE);
    if (!logFile) {
      Serial.println("Failed to open log file.");
      Serial2.println("sdc=0")
    }
    else
    {
      SD_state = 1;
      Serial2.println("sdc=1");
    }
  }
}

void logToSD(uint8_t canID, uint8_t deviceID, String data) 
{
  if (SD_state == 1 && logFile) 
  {
      logFile.print(millis());
      logFile.print(",");
      logFile.print(canID, HEX);
      logFile.print(",");
      logFile.print(deviceID, HEX);
      logFile.print(",");
      logFile.println(data);
      logFile.flush();
  }
}

void closeSD() {
    if (SD_state == 1 && logFile) {
        logFile.close();
        Serial.println("Log file closed.");
    }
}

void checkSD()
{
  if (!logFile)
  {
    SD_state = 0;
    Serial2.println("sdc=0");
  }
}
