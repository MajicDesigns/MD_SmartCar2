#include <MD_SmartCar2.h>

/**
 * \file
 * \brief Code file for SmartCar2 miscellaneous methods.
 */

void MD_SmartCar2::setLinearVelocity(int8_t vel)
{
  if (vel == 0)
    stop();
  else
    drive(vel, _vAngular);
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

  for (uint8_t i = 0; i < MAX_MOTOR; i++)
  {
    _mData[i].motor->setMaxSpeed(ppsMax);
    _mData[i].motor->setAcceleration(ppsMax / 2);
  }

  // now calculate derived constants
  _lenPerPulse = (PI * (float)_diaWheel) / (float)_ppr;   // distance traveled per encoder pulse

  _diaWheelP = _diaWheel / _lenPerPulse;   // wheel diameter converted to pulses
  _lenBaseP = _lenBase / _lenPerPulse;     // base length converted to pulses
  SCPRINT("\nWheel dia (P): ", _diaWheelP);
  SCPRINT("\nBase Len (P): ", _lenBaseP);
}
