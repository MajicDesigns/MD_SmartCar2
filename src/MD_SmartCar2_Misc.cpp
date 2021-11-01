#include <MD_SmartCar2.h>

/**
 * \file
 * \brief Code file for SmartCar2 miscellaneous methods.
 */

void MD_SmartCar2::setAccelProfile(uint16_t timebase, uint8_t steps)
{
  if (timebase == 0) timebase = 1;
  if (steps == 0) steps = 1;

  _accelTime = timebase;
  _accelSteps = steps;
}

bool MD_SmartCar2::isRunning(void)
// check if any of the motors are running
{
  bool b = true;

  for (uint8_t i = 0; i < MAX_MOTOR; i++)
    b = b && _mData[i].state != S_IDLE;

  return(b);
}

void MD_SmartCar2::setVehicleParameters(uint16_t ppr, uint16_t ppsMax, uint16_t dWheel, uint16_t lBase)
{
  // save values
  _ppr = ppr;
  _ppsMax = ppsMax;
  _diaWheel = dWheel;
  _lenBase = lBase;

  // now calculate derived constants
  _lenPerPulse = (PI * (float)_diaWheel) / (float)_ppr;   // distance traveled per encoder pulse

  _diaWheelP = _diaWheel / _lenPerPulse;   // wheel diameter converted to pulses
  _lenBaseP = _lenBase / _lenPerPulse;     // base length converted to pulses
  SCPRINT("\nWheel dia (P): ", _diaWheelP);
  SCPRINT("\nBase Len (P): ", _lenBaseP);
}
