#include "functions.h"

static int last_switch = 0;
static int last_position = LOW;
static float k = 0.1;
static int p1min = 1;
static int p2min = 7;
static int p1max = 504;
static int p2max = 761;
static float pedal1f = 1023;
static float pedal2f = 1023;
static float brakemin = 25;
static int deadzone = 30;
static int maxrpm = 550;

int pedal1r;
int pedal2r;
float pedal1s;
float pedal2s;
float acc;
int brake1r;
int brake = 0;
int cmdrpm = 0;
unsigned long cmdDelay = 100;
unsigned long sensorDiffTime = 0;
unsigned long lastCommandTime = 0;

Buzzer RTDSpeaker(SPEAKER);

void initDriverInputs()
{
  pinMode(BRAKE, INPUT);
  pinMode(PEDAL1, INPUT);
  pinMode(PEDAL2, INPUT);
  pinMode(SPEAKER, OUTPUT);
  pinMode(START, INPUT);
  pinMode(IMD_LED, OUTPUT);
  pinMode(BMS_LED, OUTPUT);

  digitalWrite(IMD_LED, LOW);
  digitalWrite(BMS_LED, LOW);
  digitalWrite(SPEAKER, LOW);
}

void getDriverInputs(unsigned long current_time)
{
  getBrake();  
    
  if (state == 0)
  {
    if (brake && digitalRead(START) == HIGH && analogRead(PEDAL1) < 100 && (current_time - last_switch) > 1000)
    {
      state = 1;
      RTDSpeaker.start(2000, 1, current_time);
      last_switch = current_time;
    }
  }
  else if (state == 1)
  {
    getThrottle(current_time);
    if (brake && digitalRead(START) == HIGH && (current_time - last_switch) > 1000)
    {
      state = 0;
      last_switch = current_time;
    }
  }
  
  digitalWrite(IMD_LED, IMD_state ? HIGH : LOW);
  digitalWrite(BMS_LED, BMS_state ? HIGH : LOW);
  RTDSpeaker.update();
}

void getThrottle(unsigned long current_time)
{
  cmdrpm = map(analogRead(PEDAL1), 0, 1023, 0, maxrpm) * 32767 / 5500; // Scales sensor inputs to rpm
  if (cmdrpm < 30)
  {
    cmdrpm = 0;
  }
  if (millis() - lastCommandTime > cmdDelay)
  {
    sendCommand(0x31, cmdrpm);
    lastCommandTime = millis();
  }
  /*
  // Raw Analog Input Values
  pedal1r = analogRead(PEDAL1);
  pedal2r = analogRead(PEDAL2);

  if((pedal1r < p1min || pedal1r > p1max) || (pedal2r < p2min || pedal2r > p2max)){
    //plausibilityError();
  }

  // Scaled analog input values
  pedal1s = map(pedal1r, p1min, p1max, 0, 1023); // The first sensor has outputs of 0-3.3V
  pedal2s = map(pedal2r, p2min, p2max, 0, 1023); // The first sensor has outputs of 0.7-3.3V from the voltage divider

  //Filtered inputs
  pedal1f = k * pedal1s + (1.0 - k) * pedal1f;
  pedal2f = k * pedal2s + (1.0 - k) * pedal2f;

  int acc = (pedal1f + pedal2f)/2;  // Command is based on average of the two sensors
  cmdrpm = map(acc, 0, 1023, 0, maxrpm) * 32767 / 5500; // Scales sensor inputs to rpm
  if(cmdrpm <= deadzone){
    cmdrpm = 0;
  }
  */
  if (cmdrpm > 0 && brake)
  {
    plausibilityError();
  }
  /*
  if((((float)(pel 
   \pedal1s - pedal2s) / 1023 )> 0.1 || ((float)(pedal1s - pedal2s) / 1023) < -0.1))
  {
    if(sensorDiffTime == 0)
    {
      sensorDiffTime = current_time;
    }
  }
  else
  {
    sensorDiffTime = 0;
  }
  if(current_time - sensorDiffTime > 100 && sensorDiffTime != 0){
      //plausibilityError();
  }*/
}

void getBrake()
{
  int brake_level = analogRead(BRAKE) /*+ analogRead(BRAKE2)) / 2*/;

  brake = brake_level > brakemin;
  // digitalWrite(BRAKE_LIGHT, brake ? HIGH : LOW);
  // write to can to send brake signal to rear stm
}
