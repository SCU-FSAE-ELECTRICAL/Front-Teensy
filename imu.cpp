#include <Wire.h>
#include "functions.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

static Adafruit_BNO055 bno = Adafruit_BNO055(55);

static float ax_, ay_, az_;
static float yaw_, pitch_, roll_;

void initIMU() {
  if (!bno.begin()) {
    Serial.println("BNO055 not detected");
    return;  
  }

  delay(100);
  bno.setExtCrystalUse(true);   
}

bool readIMU() {
  sensors_event_t accel;
  sensors_event_t orient;

  bno.getEvent(&accel, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&orient, Adafruit_BNO055::VECTOR_EULER);

  ax_ = accel.acceleration.x;
  ay_ = accel.acceleration.y;
  az_ = accel.acceleration.z;

  yaw_   = orient.orientation.x;  
  pitch_ = orient.orientation.y;
  roll_  = orient.orientation.z;

  return true;
}

float imuAx() { return ax_; }
float imuAy() { return ay_; }
float imuAz() { return az_; }

float imuYaw()   { return yaw_; }
float imuPitch() { return pitch_; }
float imuRoll()  { return roll_; }
