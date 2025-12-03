#include <MPU6050.h>

#ifndef TEMPS_H
#define TEMPS_H

// Use 'static const' to prevent multiple definition errors
static const int MotorTempData[24][2] = {
    {980,  -55},
    {1030, -50},
    {1135, -40},
    {1247, -30},
    {1367, -20},
    {1495, -10},
    {1630, 0},
    {1772, 10},
    {1922, 20},
    {2000, 25},
    {2080, 30},
    {2245, 40},
    {2417, 50},
    {2597, 60},
    {2785, 70},
    {2980, 80},
    {3182, 90},
    {3392, 100},
    {3607, 110},
    {3817, 120},
    {3915, 125},
    {4008, 130},
    {4166, 140},
    {4280, 150}
};

static const int ControllerTempData[33][2] = {
    {16308, -30},
    {16387, -25},
    {16487, -20},
    {16609, -15},
    {16757, -10},
    {16938, -5},
    {17151, 0},
    {17400, 5},
    {17688, 10},
    {18017, 15},
    {18387, 20},
    {18797, 25},
    {19247, 30},
    {19733, 35},
    {20250, 40},
    {20793, 45},
    {21357, 50},
    {21933, 55},
    {22515, 60},
    {23097, 65},
    {23671, 70},
    {24232, 75},
    {24775, 80},
    {25296, 85},
    {25792, 90},
    {26261, 95},
    {26702, 100},
    {27114, 105},
    {27497, 110},
    {27851, 115},
    {28179, 120},
    {28480, 125}
};

// Use 'inline' to prevent multiple definition errors
inline float getMotorTemp(float value){
  int high = 23;
  int low = 0;
  int mid = 0;
  float res = ((float)value * 4000)/(32000 - value);

  //Binary Search
  while(low+1 < high){
    mid = floor((high + low)/2);
    if (res == MotorTempData[mid][0]){
      return MotorTempData[mid][1];
    }
    else if (res < MotorTempData[mid][0]){
      high = mid - 1;
    }
    else if (res > MotorTempData[mid][0]){
      low = mid + 1;
    }
  }
  //Linear Interpolation
  return (float)MotorTempData[low][1] + (res - MotorTempData[low][0]) * ((float)(MotorTempData[high+1][1] - MotorTempData[low][1]) / (MotorTempData[high+1][0] - MotorTempData[low][0]));
}

// Use 'inline' to prevent multiple definition errors
inline float getControllerTemp(float val){
  int high = 32;
  int low = 0;
  int mid = 0;

  //Binary Search
  while(low < high){
    mid = floor((high + low)/2);
    if (val == ControllerTempData[mid][0]){
      return ControllerTempData[mid][1];
    }
    else if (val < ControllerTempData[mid][0]){
      high = mid - 1;
    }
    else if (val > ControllerTempData[mid][0]){
      low = mid + 1;
    }
  }
  //Linear Interpolation
  return (float)ControllerTempData[low][1] + (val - ControllerTempData[low][0]) * ((float)(ControllerTempData[high+1][1] - ControllerTempData[low][1]) / (ControllerTempData[high+1][0] - ControllerTempData[low][0]));
}


#endif // TEMPS_H
