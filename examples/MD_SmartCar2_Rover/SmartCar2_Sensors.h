#pragma once
// Encapsulates all vehicle sensors into one class
//
// NewPing library available from https://bitbucket.org/teckel12/arduino-new-ping/src/master/
//

#include <NewPing.h>
#include "SmartCar_HW.h"

class cSensors
{
public:
  typedef enum { FLL, FL, FC, FR, FRR, BR, BL } direction_t; 

  cSensors(void)
  {
    bumpers = 0;
    for (uint8_t i = 0; i < MAX_SONAR; i++)
      sonar[i] = false;
  }

  void begin(void) 
  {
    pinMode(PIN_R_BUMPER, INPUT_PULLUP);
    pinMode(PIN_L_BUMPER, INPUT_PULLUP);
  }

  void read(void) 
  {
    _newData = false;     // will be set if any updates happen
    readBumper();
    readSonar();
    readLight();
  }

  inline bool isUpdated(void) { return(_newData); }

  bool getBumper(direction_t dir)
  {
    uint8_t bit;

    switch(dir)
    {
      case FLL: bit = 0; break;
      case FL:  bit = 1; break;
      case FC:  bit = 2; break;
      case FR:  bit = 3; break;
      case FRR: bit = 4; break;
      case BL:  bit = 5; break;
      case BR:  bit = 6; break;
      default:  return(false);
    }
    return((bumpers & (1 << bit)) != 0);
  }

  uint8_t getSonar(direction_t dir)
  {
    switch (dir)
    {
      case FL: return(sonar[0]);
      case FC: return(sonar[1]);
      case FR: return(sonar[2]);
      default: return(0);
    }
  }

  void dump(Stream& S)
  {
    if (isUpdated())
    {
      S.print("\n");
      S.print("B("); S.print(bumperL); S.print(','); S.print(bumperR);
      S.print(") S("); S.print(sonarL); S.print(','); S.print(sonarM); S.print(','); S.print(sonarR);
      S.print(") L("); S.print(lightL); S.print(','); S.print(lightR);
      S.print(")");
    }
  }

private:
  // Available Sensor Data
  uint16_t bumper;              // Bump switches (1 bit per switch)
  uint16_t sonar[MAX_SONAR];    // Sonar data (0 -> distance > MAX_DISTANCE)

  // Bumper definitions
  uint32_t _lastBumperPoll;
  bool _newData;

  void readBumper(void)
  {
    if (millis() - _lastBumperPoll >= BUMPER_POLL_PERIOD)
    {
      bool bl = (digitalRead(PIN_L_BUMPER) == LOW);
      bool br = (digitalRead(PIN_R_BUMPER) == LOW);

      _newData = _newData || (bumperL != bl) || (bumperR != br);
      bumperL = bl;
      bumperR = br;
      _lastBumperPoll = millis();
    }
  }

  // Sonar (Ping) definitions
  static const uint8_t MAX_SONAR = 3;             // Number of ping sensors
  uint8_t _curSonar;        // next device to poll
  uint32_t _lastSonarPoll;

  NewPing _sonar[MAX_SONAR] =
  {
    NewPing(PIN_L_SONAR, PIN_L_SONAR, DIST_MAX),
    NewPing(PIN_M_SONAR, PIN_M_SONAR, DIST_MAX),
    NewPing(PIN_R_SONAR, PIN_R_SONAR, DIST_MAX)
  };

  void readSonar(void)
  {
    if (millis() - _lastSonarPoll > SONAR_POLL_PERIOD)
    {
      uint16_t ping = _sonar[_curSonar].ping_cm();

      // error checking
      if (ping == 0) ping = DIST_ALLCLEAR;    // application distance comparisons work better with non zero

      // handle the data
      _newData = _newData || (sonar[_curSonar] != ping);
      sonar[_curSonar] = ping;
      _curSonar++;

      // set up for next pass
      if (_curSonar >= MAX_SONAR) _curSonar = 0;    // roll over
      _lastSonarPoll = millis();
    }
  }
};

cSensors Sensors;     // declare one instance!