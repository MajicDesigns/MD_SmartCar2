// Minimal MD_CleverCar implementation
// 
// Stop for TIME_PERIOD, drive in a circle for TIME_PERIOD, repeat.

#include <MD_SmartCar2.h>
#include "SmartCar2_HW.h"

// ------------------------------------
// Global Variables
// Initialize with pin sequence IN1-IN2-IN3-IN4 for using the MD_Stepper with 28BYJ-48
MD_Stepper MR(PIN_INA1, PIN_INA2, PIN_INA3, PIN_INA4);
MD_Stepper ML(PIN_INB2, PIN_INB1, PIN_INB4, PIN_INB3);

MD_SmartCar2 Car(&ML, &MR); 

void setup(void)
{
  Car.begin(PPR, PPS_MAX, DIA_WHEEL, LEN_BASE);
}

void loop(void)
{
  static const uint32_t TIME_RUN = 12000;   // in ms
  static const uint32_t TIME_PAUSE = 5000;  // in ms

  static int8_t angularV = 30;    // degrees per second
  static int8_t velocity = 100;   // % full speed
  static uint32_t timeLast;
  static bool inPause = true;
  static uint32_t curTimer = (inPause ? TIME_PAUSE : TIME_RUN);

  if (millis() - timeLast >= curTimer)
  {
    if (inPause)
    {
      Car.drive(velocity, angularV);    // move the car
      curTimer = TIME_RUN;
    }
    else
    {
      Car.stop();                       // stop the car
      curTimer = TIME_PAUSE;
    }
    timeLast = millis();
    inPause = !inPause;
  }

  Car.run();
}
