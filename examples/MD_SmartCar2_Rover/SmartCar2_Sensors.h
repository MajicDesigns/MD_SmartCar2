#pragma once
// Encapsulates all vehicle sensors into one class
//
// - Single ping sensor on a servo to swing it around 5 predetermined positions
// - Bumper switches connected to a PCF8574 I2C I/O extender
// 
// External library dependencies are listed in the main application.
//

// Options for Scan sensors
#define USE_HCSR04    1   // Ultrasonic ping sensor
#define USE_VL53L0X   0   // Laser Time of Flight sesnor

#include <Wire.h>
#include <ServoTimer2.h>
#include <PCF8574.h>
#if USE_HCSR04
#include <NewPing.h>
#endif
#if USE_VL53L0X
#include <VL53L0X.h>
#endif
#include "SmartCar2_HW.h"

#define SR_DETAIL 0     // print out scan detailed debug to serial monitor

class cSensors
{
public:
  // These are the bumpers as bit numbers in the byte from the PCF8574 I/O
  typedef enum { BR = 0, BC = 1, BL = 2, FRR = 3, FR = 4, FC = 5, FL = 6, FLL = 7, MAX_BUMPER = 8 } bumpDir_t; 
  // These are the scan positions, also used as indexes into the scan positioning array
  typedef enum { SLL = 0, SL = 1, SC = 2, SR = 3, SRR = 4, MAX_SCAN = 5 } scanDir_t;

  cSensors(void) { }

  void begin(void) 
  {
    Wire.begin();
    beginBumper();
    beginScan();
    beginBattery();
  }

  void read(void) 
  // read all the sensor data
  {
    _newData = false;     // will be set if any updates happen
    readBumper();
    readScan();
    readBattery();
  }

  inline bool isUpdated(void) { return(_newData); }   // return true if there has been a data update in the last read()

  inline float getVoltage(void) { return(_volts); }   // return the last read voltage

  inline bool    getBumper(bumpDir_t dir) { return((_bumpers & _BV(dir)) != 0); } // return a specific bumper position
  inline uint8_t getBumperAll(void)       { return(_bumpers); }                   // return all bumper bits in the byte

  inline void enableScan(bool b)        { _enableScan = b; resetScan(); }      // enable/disable the scan reading
  inline void setScanPeriod(uint16_t s) { if (s >= SCAN_FAST_POLL) _scanPeriod = s; }  // set the new time
  inline bool isScanEnabled(void)       { return (_enableScan); }               // return current status
  inline uint8_t getScan(scanDir_t dir) { return(_scanPoint[dir].data); }             // get a specific scan reading

  uint16_t getSweepTime(void)
  // Calculate the time in ms it takes for one servo sweep (servo move 
  // and scan time) for the currently selected scanning pattern and scan
  // period.
  {
    uint16_t time = _scanPeriod;

    for (uint8_t i = 0; i < MAX_SCAN; i++)
      if (_scanMask & (1 << i)) 
        time += _scanPeriod;

    return(time);
  }

  void setScanMask(uint8_t newMask)
  // The mask determines which scan positions get scanned
  {
#if SR_DETAIL
    Serial.print(F("\nNew Mask: 0x"));
    Serial.print(newMask);
#endif
    if (newMask == 0)  // need to do at least one of them!
      newMask = (1 << SC);

    if (newMask != _scanMask)
    {
      _scanMask = newMask;
      for (uint8_t i = 0; i < MAX_SCAN; i++)
        if ((_scanMask & _BV(i)) == 0)
          _scanPoint[i].data = DIST_ALLCLEAR;
    }
  }

  void dump(Stream& S)
  // debugging dump of the sensor data to the output stream S
  {
    S.print("\n");
    S.print("V(");
    S.print(_volts, 1);

    S.print(") B("); 
    for (uint8_t i = 0; i < MAX_BUMPER; i++)
      S.print((_bumpers >> i) & 1);

    S.print(") S("); 
    for (uint8_t i = 0; i < MAX_SCAN; i++)
    {
      S.print(_scanPoint[i].data);
      S.print(',');
    }

    S.print(")");
  }

private:
  bool _newData;

  //-------------------------------
  // Battery monitor definitions
  float _volts;   // battery voltage in 1/0 volts
  uint32_t _lastBatteryPoll;

  void beginBattery(void)
  {
    pinMode(PIN_VOLTAGE, INPUT);
    readBattery(true);
  }

  void readBattery(bool force = false)
  // Battery Voltage maxc is V_FULL. This is fed through a voltage divider
  // so that V_FULL is converted to 5V max, which can then be fed into
  // an Arduino input pin.
  {
    if (force || millis() - _lastBatteryPoll >= V_POLL_PERIOD)
    {
      // Read the analog value and convert reading to a voltage 
      uint16_t v = analogRead(PIN_VOLTAGE);

      // Convert the analog reading (0 - 1023) to a voltage (0 - V_FULL):
      _volts = v * (V_FULL / 1023.0);
    }
  }

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
  // Scan (distance) definitions
  // The scanner uses a distance measuring sensor and a servo to rotate it.

  struct scanPoint_t    // record for a scanning point
  {
    uint8_t heading;    // direction to set the servo
    uint8_t data;       // the data read for this direction
  };
  
  scanPoint_t _scanPoint[MAX_SCAN] =
  {
    { 180, DIST_ALLCLEAR },   // SLL
    { 135, DIST_ALLCLEAR },   // SL
    {  90, DIST_ALLCLEAR },   // SC
    {  45, DIST_ALLCLEAR },   // SR
    {   0, DIST_ALLCLEAR },   // SRR
  };

  bool _enableScan;           // Scanning is enabled or not
  uint8_t _curPoint;          // Current point being scanned
  uint8_t _scanMask;          // mask for positions to include in the sweep
  uint16_t _scanPeriod;       // time between scans
  uint32_t _lastScanPoll;     // millis() time marker for last scan poll

#if USE_HCSR04
  NewPing _scan = NewPing(PIN_SCAN, PIN_SCAN, DIST_MAX);
#endif
#if USE_VL53L0X
  VL53L0X _scan;
#endif
  ServoTimer2 _servo;

  void resetScan(void)
  // reset the scan subsystem to default starting conditions
  {
    _scanPeriod = SCAN_SLOW_POLL;
    _curPoint = SC;
    setScanMask(_BV(_curPoint));
    moveServo(_scanPoint[_curPoint].heading);

    for (uint8_t i = 0; i < MAX_SCAN; i++)
      _scanPoint[i].data = DIST_ALLCLEAR;
  }

  void beginScan(void)
  // initialize the scan subsystem
  {
    bool b = true;

    _servo.attach(PIN_SERVO);
#if USE_VL53L0X
    b = _scan.init();
    if (b) _scan.startContinuous();
#endif
    resetScan();
    enableScan(b);
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

  void readScan(void)
  // Read data from the scan subsystem if it is currently enabled.
  // Get the reading at the current position, then move to the next 
  // position.  The next position is determined by the mask set by 
  // setScanMask(). This motion should be completed by the time when 
  // we come back for the next distance reading. Scan sweep is in 
  // one direction only, with a swing back to start again.
  {
    if (!_enableScan) return;
    
    if (millis() - _lastScanPoll > _scanPeriod)
    {
      uint16_t dist;
#if USE_HCSR04
      dist = _scan.ping_cm();
#endif
#if USE_VL53L0X
      dist = _scan.readRangeContinuousMillimeters() / 10;
#endif

      // error checking
      if (dist == 0) dist = DIST_ALLCLEAR;    // application distance comparisons work better with non zero

      // handle the data
      _newData = _newData || (_scanPoint[_curPoint].data != dist);
      _scanPoint[_curPoint].data = dist;
#if SR_DETAIL
      Serial.print("\nS["); Serial.print(_curPoint);
      Serial.print("]="); Serial.print(_scanPoint[_curPoint].data);
#endif

      do
      {
        // Set up for next pass
        _curPoint++;          // set the next to scan
#if SR_DETAIL
//        Serial.print("\nN "); Serial.print(_curPoint); 
//        Serial.print(" M 0x"); Serial.print(_scanMask, HEX);
#endif
        // Reverse swing direction at end of pass
        if (_curPoint >= MAX_SCAN) 
          _curPoint = 0;      // rest to start of sequqnce again
      } while ((_scanMask & (1 << _curPoint)) == 0);
#if SR_DETAIL
      Serial.print(" --> "); Serial.print(_curPoint);
#endif

      // Start the servo movement so that it goes to new heading 
      // while we are waiting between pings.      
      moveServo(_scanPoint[_curPoint].heading);    // move to new position
      _lastScanPoll = millis();         // new timer starting point
    }
  }
};

cSensors Sensors;     // declare the only instance of this class!