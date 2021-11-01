#pragma once
// Encapsulates all vehicle sensors into one class
//
// - Single ping sensor on a servo to swing it around 5 predetermined positions
// - Bumper switches connected to a PCF8574 I2C I/O extender
// 
// External library dependencies are listed in the main application.
//

#include <NewPing.h>
#include <ServoTimer2.h>
#include <PCF8574.h>
#include "SmartCar2_HW.h"

#define SR_DETAIL 0     // print out sonar detailed debug to serial monitor

class cSensors
{
public:
  // These are the bumpers as bit numbers in the byte from the PCF8574 I/O
  typedef enum { BR = 0, BC = 1, BL = 2, FRR = 3, FR = 4, FC = 5, FL = 6, FLL = 7, MAX_BUMPER = 8 } bumpDir_t; 
  // These are the sonar positions, also used as indexes into the sonar positioning array
  typedef enum { SLL = 0, SL = 1, SC = 2, SR = 3, SRR = 4, MAX_SONAR = 5 } scanDir_t;

  cSensors(void) { }

  void begin(void) 
  {
    beginBumper();
    beginSonar();
  }

  void read(void) 
  // read all the sensor data
  {
    _newData = false;     // will be set if any updates happen
    readBumper();
    readSonar();
  }

  inline bool isUpdated(void) { return(_newData); }   // return true if there has been a data update in the last read()

  inline bool    getBumper(bumpDir_t dir) { return((_bumpers & _BV(dir)) != 0); } // return a specific bumper position
  inline uint8_t getBumperAll(void)       { return(_bumpers); }                   // return all bumper bits in the byte

  inline void enableSonar(bool b)        { _enableSonar = b; resetSonar(); }      // enable/disable the sonar reading
  inline bool isSonarEnabled(void)       { return (_enableSonar); }               // return current status
  inline uint8_t getSonar(scanDir_t dir) { return(_sonarData[dir]); }             // get a specific sonar reading

  uint16_t getSonarScanTime(void)
  // Calculate the time in ms it takes for one sonar scan (servo move 
  // and scan time) for the currently selected scanning pattern.
  {
    uint16_t time = 0;

    for (uint8_t i = 0; i < MAX_SONAR; i++)
      if (_pingMask & (1 << i)) 
        time += SONAR_POLL_PERIOD;

    return(time * 2);
  }

  void setSonarMask(uint8_t newMask)
  // The mask determines which sonar positions get scanned
  {
#if SR_DETAIL
    Serial.print(F("\nNew Mask: 0x"));
    Serial.print(newMask);
#endif
    if (newMask == 0) newMask = (1 << SC);  // only do the center one
    _pingMask = newMask;
    for (uint8_t i = 0; i < MAX_SONAR; i++)
      if (!(newMask & (1 << i)))
        _sonarData[i] = 0;
  }

  void dump(Stream& S)
  // debugging dump of the sensor data to the output stream S
  {
    S.print("\n");
    S.print("B("); 
    for (uint8_t i = 0; i < MAX_BUMPER; i++)
      S.print((_bumpers >> i) & 1);
    S.print(") S("); 
    for (uint8_t i = 0; i < MAX_SONAR; i++)
    {
      S.print(_sonarData[i]);
      S.print(',');
    }
    S.print(")");
  }

private:
  bool _newData;

  //-------------------------------
  // Bumper definitions
  uint8_t _bumpers;              // Bump switches (1 bit per switch)
  PCF8574 pcf = PCF8574(PCF8574_ADDR);
  uint32_t _lastBumperPoll;

  void beginBumper(void)
  // initialize the bumper subsystem
  {
    _bumpers = 0;
    pcf.begin();
  }

  void readBumper(void)
  // read data from the bumper subsystem
  {
    if (millis() - _lastBumperPoll >= BUMPER_POLL_PERIOD)
    {
      uint8_t b = ~pcf.read8();   // convert PULLUP values to standard logic

      _newData = _newData || (_bumpers != b);
      _bumpers = b;
      _lastBumperPoll = millis();
    }
  }

  //-------------------------------
  // Sonar (Ping) definitions
  bool _enableSonar;                    // Sonar is enabled or not
  uint16_t _sonarData[MAX_SONAR];       // Sonar values data SLL, SL, ... SRR
  int8_t _curSonar;                     // next direction to process
  int8_t _scanDirection = 1;            // scanning direction
  uint8_t _heading[MAX_SONAR] = { 180, 135, 90, 45, 0 };    // sonar readings directions SLL, SL, SC, SR, SRR
  uint8_t _pingMask;
  uint32_t _lastSonarPoll;

  NewPing _ping = NewPing(PIN_SONAR, PIN_SONAR, DIST_MAX);
  ServoTimer2 _servo;

  void resetSonar(void)
  // reset the sonar subsystem to default starting conditions
  {
    _scanDirection = 1;
    setSonarMask(0);
    _curSonar = SC;
    moveServo(_heading[_curSonar]);

    for (uint8_t i = 0; i < MAX_SONAR; i++)
      _sonarData[i] = DIST_ALLCLEAR;
  }

  void beginSonar(void)
  // initialize the sonar subsystem
  {
    _servo.attach(PIN_SERVO);
    enableSonar(true);
  }

  void moveServo(uint16_t pos)
  // move the servo to pos
  // pos is the angle in degrees, servo moves in pulse width, so convert
  {
    if (pos < 0) pos = 0;
    else if (pos > 180) pos = 180;

    // convert to pulse width
    pos = map(pos, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    _servo.write(pos);
  }

  void readSonar(void)
  // Read data from the sonar subsystem if it is currently enabled.
  // Get the reading at the current position, then move to the next 
  // position. This move will be completed when we come back for the next
  // sonar poll. The next position is determined by the mask set by 
  // setSonarMask(). Scans are done in one direction then swing back in 
  // the reverse, so the inner positions of a scan will always get scanned 
  // twice as often as the outer extremes.
  {
    if (!_enableSonar) return;
    
    if (millis() - _lastSonarPoll > SONAR_POLL_PERIOD)
    {
      uint16_t dist = _ping.ping_cm();

      // error checking
      if (dist == 0) dist = DIST_ALLCLEAR;    // application distance comparisons work better with non zero

      // handle the data
      _newData = _newData || (_sonarData[_curSonar] != dist);
      _sonarData[_curSonar] = dist;

      do
      {
        // Set up for next pass
        _curSonar += _scanDirection;          // set the next to scan
#if SR_DETAIL
        Serial.print("\nN "); Serial.print(_curSonar); 
        Serial.print(" M 0x"); Serial.print(_pingMask, HEX);
#endif
        // Reverse swing direction at end of pass
        if (_curSonar >= MAX_SONAR) 
        { 
#if SR_DETAIL
          Serial.print(" RM");
#endif
          _curSonar = MAX_SONAR; // next pass will decrement first
          _scanDirection = -1; 
        }
        else if (_curSonar <= 0) 
        { 
#if SR_DETAIL
          Serial.print(" R0");
#endif
          _curSonar = 0;        // next pass will increment first
          _scanDirection = 1; 
        }
      } while ((_pingMask & (1 << _curSonar)) == 0);
#if SR_DETAIL
      Serial.print(" ----> "); Serial.print(_curSonar);
#endif

      // Start the servo movement so that it goes to new heading 
      // while we are waiting between pings.      
      moveServo(_heading[_curSonar]);    // move to new position
      _lastSonarPoll = millis();         // new timer starting point
    }
  }
};

cSensors Sensors;     // declare the only instance of this class!