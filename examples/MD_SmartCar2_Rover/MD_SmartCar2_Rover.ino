// Application to control an autonomous MD_SmartCar2 vehicle using a 
// Bluetooth interface implemented with a HC-05 module that has been 
// pre-initialized and paired to the BT master.
// 
// This is an example of behaviors built on top of the library infrastructure.
// High level control of the robotic vehicle is modeled after 'Behavior Based 
// Robotics' book.
//
// All vehicle behavior types can be exercised and monitored from the AI2 
// 'SmartCar2_Rover_Control' interface application found with this code.
//
// SmartCar2_HW.h contains all the phuysical constants and hadware I/O 
// pin definitions.
// 
// External library dependencies:
// - PCF8574 library available from https://github.com/RobTillaart/PCF8574.git or the IDE Library Manager
// 
// - AltSoftSerial is available from https://github.com/PaulStoffregen/AltSoftSerial or the IDE Library Manager
// Note: AltSoftSerial is hard coded to use digital pins 8 and 9 on the Arduino Nano/Uno.
// 
// - NewPing library available from https://bitbucket.org/teckel12/arduino-new-ping/src/master/ or the IDE Library Manager
// Note: In NewPing.h TIMER_ENABLED must be turned off.
// 
// - VL53L0X library is available from https://github.com/pololu/vl53l0x-arduino or the IDE Library Manager
// 
// - ServoTimer2 library is available from https://github.com/nabontra/ServoTimer2
// Note: In ServoTimer2.h change MIN_PULSE_WIDTH to 500 and MAX_PULSE_WIDTH to 2500. 
//       This enables 0 to 180 degree swing in the servo.
//

#include <AltSoftSerial.h>
#include <MD_SmartCar2.h>
#include <MD_cmdProcessor.h>
#include "SmartCar2_HW.h"
#include "SmartCar2_Sensors.h"
#include "SmartCar2_Buzzer.h"

#ifndef ENABLE_DEBUG       
#define ENABLE_DEBUG 0     // set to 1 to enable serial debugging info
#endif

#ifndef REMOTE_START
#define REMOTE_START 1     // set to 1 to start from external interface, other starts on reset
#endif

#ifndef DUMP_SENSORS
#define DUMP_SENSORS 0     // set to 1 to echo sensor data to Serial Monitor
#endif

#ifndef ENABLE_TELEMETRY   
#define ENABLE_TELEMETRY !ENABLE_DEBUG // set to 1 for BT or debug telemetry transmission
#endif

// Serial debugging macros
#if ENABLE_DEBUG
#define CMDStream Serial
#define DEBUG(s,v)   do { Serial.print(F(s)); Serial.print(v); } while (false)
#define DEBUGX(s,v)  do { Serial.print(F(s)); Serial.print(F("0x")); Serial.print(v, HEX); } while (false)
#define DEBUGS(s)    do { Serial.print(F(s)); } while (false)
#else
#define CMDStream BTSerial
#define DEBUG(s,v)
#define DEBUGX(s,v)
#define DEBUGS(s)
#endif

// ------------------------------------
// Global states
bool runEnabled = (REMOTE_START == 0);
bool restart = true;

enum behavior_t 
{
  ESCAPE = 0,     // emergency bumping into things or way too close. Always gets priority.
  AVOID = 1,      // CRUISE selected, moves to best open space when it gets too close to obstacle.
  WALLFOLLOW = 2, // follow a wall.
  CRUISE = 3,     // default operating when nothing else is running - move straight.
};

behavior_t defaultBehavior = CRUISE;
behavior_t runBehavior = defaultBehavior;
behavior_t prevBehavior = runBehavior;

// ------------------------------------
// Global Variables
// Initialize with pin sequence IN1-IN2-IN3-IN4 for using MD_Stepper with 28BYJ-48.
// Motors should be wired identically and the change of direction happens in pin order below.
MD_Stepper MR(PIN_INA1, PIN_INA2, PIN_INA3, PIN_INA4);  // rotate in fwd direction and ...
MD_Stepper ML(PIN_INB2, PIN_INB1, PIN_INB4, PIN_INB3);  // ... reverse (opposite) direction

MD_SmartCar2 Car(&ML, &MR);                     // SmartCar2 control object
AltSoftSerial BTSerial(PIN_BT_RX, PIN_BT_TX);   // BT comms link
cBuzzer Buzzer(PIN_BUZZER);

// ------------------------------------
// BT monitoring telemetry
#if ENABLE_TELEMETRY
#if ENABLE_DEBUG
#define TELStream Serial
#else // not debug
#define TELStream BTSerial
#endif
#define TEL_VALUE(s,v) do { TELStream.print(F(s)); TELStream.print(v); } while (false)
#define TEL_MESG(s)    do { TELStream.print(F(s)); } while (false)
#define TEL_ALARM(s)   do { TELStream.print(F("\nALARM: ")); TELStream.print(F(s)); \
                            TELStream.print(F("\n")); Buzzer.setMode(cBuzzer::ALARM); \
                          } while (false)
#define TEL_PANIC(s)   do { TELStream.print(F("\nPANIC: ")); TELStream.print(F(s)); \
                            TELStream.print(F("\n")); Buzzer.setMode(cBuzzer::PANIC); \
                          } while (false)
#if ENABLE_DEBUG
#define TEL_PACKET(s)
#else // not debug
#define TEL_PACKET(s)  do { TELStream.print('$'); TELStream.print(s); TELStream.print('~'); } while (false)
#endif
#else // not telemetry
#define TEL_VALUE(s,v)
#define TEL_MESG(s)
#define TEL_PACKET(s)
#define TEL_ALARM(s)
#define TEL_PANIC(s)
#endif

// ------------------------------------
// Scan scanning groups
// These are bit masks that define related sets of scan scanning positions in
// different related sets that we may want to use.
//
const uint8_t SCAN_MID = (_BV(cSensors::SL) | _BV(cSensors::SC) | _BV(cSensors::SR));
const uint8_t SCAN_CTR = _BV(cSensors::SC);
const uint8_t SCAN_LEFT  = (_BV(cSensors::SLL) | _BV(cSensors::SL) | _BV(cSensors::SC));
const uint8_t SCAN_RIGHT = (_BV(cSensors::SC) | _BV(cSensors::SR) | _BV(cSensors::SRR));
const uint8_t SCAN_ALL = (SCAN_LEFT | SCAN_MID | SCAN_RIGHT);
const uint8_t SCAN_WALL = (_BV(cSensors::SLL) | _BV(cSensors::SC));

// ------------------------------------
// Bump switch sensing
// These are bit masks representing related sets of bump switch positions 
// that we can use to determine collison directions.
const uint8_t BUMP_LEFT  = _BV(cSensors::FLL) | _BV(cSensors::FL);
const uint8_t BUMP_MID   = _BV(cSensors::FL) | _BV(cSensors::FC) | _BV(cSensors::FR);
const uint8_t BUMP_RIGHT = _BV(cSensors::FR) | _BV(cSensors::FRR);
const uint8_t BUMP_FRONT = BUMP_LEFT | BUMP_MID | BUMP_RIGHT;
const uint8_t BUMP_BACK = _BV(cSensors::BL) | _BV(cSensors::BC) | _BV(cSensors::BR);

// ------------------------------------
// cmdProcessor Handler functions
// The external control app uses the cmdProcessor to control the SmartCar.
// If debug is turned on this is switched to the Serial Monitor.
//

bool checkHealth(void)
// Check the health of the system and return true if all ok
{
  bool b = true;

  if (Sensors.getVoltage() < V_ALARM)
  {
    TEL_ALARM("Battery LOW");
    DEBUG("\nV Check: ", Sensors.getVoltage());
    b = false;
  }

  return(b);
}

void stopVehicle(void)
// stop the vehicle and keep the buzzer working in current mode
{
  Car.stop();
  Sensors.setScanMask(SCAN_CTR);
  runBehavior = defaultBehavior;
  runEnabled = false;
  TEL_MESG("\n>> STOP <<");
}

void handlerM(char* param)
// Change running mode
{
  DEBUGS("Mode ");
  switch (*param)
  {
  case '1': runBehavior = WALLFOLLOW; DEBUGS("WALL\n");   break;
  default:  runBehavior = CRUISE;     DEBUGS("CRUISE\n"); break;
  }

  defaultBehavior = runBehavior;    // set the default so we remember
}

void handlerR(char* param) 
// Run on or off
{ 
  runEnabled = (*param == '1'); 
  DEBUG("Run ", runEnabled);  DEBUGS("\n");

  if (runEnabled)
  {
    if (checkHealth())
    {
      restart = true;
      Buzzer.setMode(cBuzzer::HEARTBEAT);
      Car.setLinearVelocity(SPEED_CRUISE);
      TEL_MESG("\n>> RUN <<");
    }
    else
      stopVehicle();
  }
  else
  {
    stopVehicle();
    Buzzer.setMode(cBuzzer::SILENT);
  }
}

void handlerZ(char* param)
// Sound on or off
{
  Buzzer.enable(*param == '1');
  DEBUG("Sound ", *param);  DEBUGS("\n");
}

void handlerS(char* param)
// Run on or off
{
  bool enabled = (*param == '1');

  DEBUG("Scan ", enabled);  DEBUGS("\n");
  Sensors.enableScan(enabled);
}

void handlerH(char* param);   // prototype for cmdTable

const MD_cmdProcessor::cmdItem_t PROGMEM cmdTable[] =
{
  { "?", handlerH,  "", "Help text", 1 },
  { "r", handlerR, "b", "Run 0=stop, 1=start", 1 },
  { "m", handlerM, "m", "Mode 0=Cruise, 1=Wall", 1 },
  { "s", handlerS, "b", "Scan 0=disable, 1=enable" },
  { "z", handlerZ, "b", "Sound 0=disable, 1=enable"}
};

MD_cmdProcessor CP(CMDStream, cmdTable, ARRAY_SIZE(cmdTable), true);

void handlerH(char* param) {  CP.help(); }

// ------------------------------------
// Telemetry related functions
// Telemetry is sent to the control application when the data changes or 
// at least every TELEMTRY_PERIOD.
// The packet is an ASCII string as defined in senTelemetryData().
//

inline uint8_t bool2ASCII(char* p, bool state, uint8_t size)
// Place a bool starting at *p, one character long.
// Return the number of inserted characters.
{
  for (uint8_t i = 0; i < size; i++)
    *p++ = (state ? '1' : '0');

  return(size);
}

uint8_t num2ASCII(char *p, int16_t num, uint8_t size)
// Place a num with leading zeros in size field starting at *p.
// Return the number of inserted characters.
{
  bool negative = (num < 0);

  num = abs(num);
  for (int16_t i = size - 1; i >= 0; i--)
  {
    p[i] = (num % 10) + '0';
    num = num / 10;
  }

  if (negative) *p = '-';

  return(size);
}

uint8_t float2ASCII(char* p, float num, uint8_t size, uint8_t dec)
{
  uint16_t frac = abs(num - trunc(num)) * pow(10, dec);

  num2ASCII(&p[size - dec], frac, dec);
  p[size - dec - 1] = '.';
  num2ASCII(p, (int16_t)trunc(num), size - dec - 1);

  return(size);
}

void sendTelemetryData(bool newData = false)
// Collate current info and send it as a telemetry packet.
// 
// Data Packet format is fixed length ASCII '-','0'..'9' stream with 
// a start indicator '$' and end indicator '~'.
// 
// Data contents (with start position and field width)
// 
// |Idx| Field | Description
// +---+-------+------------------------------------------------
// | 00| R     | run (1) or stopped (0)
// | 01| D     | Default Behavior (0-1) for CRUISE, WALLFOLLOW
// | 02| C     | Current Behavior (0-3) for CRUISE, WALLFOLLOW, AVOID, ESCAPE
// | 03| VVVV  | Linear velocity % FS (signed)
// | 07| AA.AA | Angular velocity in radians (3.2 float signed)
// | 12| BBB   | Bumper bitfield (decimal No) MSB [FLL|FL|FC|FR|FRR|BL|BC|BR] LSB
// | 15| FLL   | Scan FLL distance (cm)
// | 18| FL_   | Scan FL distance (cm)
// | 21| FC_   | Scan FC distance (cm)
// | 24| FR_   | Scan FR distance (cm)
// | 27| FRR   | Scan FRR distance (cm)
// | 30| BB.B  | Battery Voltage (Volts)
// | 34| S     | Packet Sequence number (0-9)
// | 35|       | End of packet
//
// NOTE: Any or additionas need to be reflected in the initializing 
// template in the declaration of the string below.
{
#if ENABLE_TELEMETRY
  static uint32_t timeLast = 0;
  static int8_t seqNo = 0;

  if (newData || (millis() - timeLast >= TELEMETRY_PERIOD))
  {
    static char mesg[] = { "RDCVVVVAA.AABBBFLLFL_FC_FR_FRRBB.BS" };
    char *p = mesg;

    // Running modes
    p += bool2ASCII(p, runEnabled, 1);

    switch (defaultBehavior)
    {
    case WALLFOLLOW: *p = '1'; break;
    default: *p = '0'; break;
    }
    p++;

    switch (runBehavior)
    {
    case WALLFOLLOW: *p = '1'; break;
    case AVOID:  *p = '2'; break;
    case ESCAPE: *p = '3'; break;
    default: *p = '0'; break;
    }
    p++;

    // Speed & Direction
    p += num2ASCII(p, Car.getLinearVelocity(), 4);
    p += float2ASCII(p, Car.getAngularVelocity(), 5, FLOAT_DECIMALS);

    // Sensor Data
    p += num2ASCII(p, Sensors.getBumperAll(), 3);

    p += num2ASCII(p, Sensors.getScan(cSensors::SLL), 3);
    p += num2ASCII(p, Sensors.getScan(cSensors::SL), 3);
    p += num2ASCII(p, Sensors.getScan(cSensors::SC), 3);
    p += num2ASCII(p, Sensors.getScan(cSensors::SR), 3);
    p += num2ASCII(p, Sensors.getScan(cSensors::SRR), 3);

    p += float2ASCII(p, Sensors.getVoltage(), 4, 1);

    // Sequence number
    p += num2ASCII(p, seqNo, 1);
    seqNo++;
    if (seqNo == 10) seqNo = 0;

    *p = '\0';          // make sure it is terminated
    TEL_PACKET(mesg);   // send it off - macro will add top and tail to the packet

    timeLast = millis();  // set up for next time
  }
#endif
}

// ------------------------------------
// Implemented High Level Behaviors
// 
// Each behavior has 2 related functions:
// 
// - activateBehavior() to test whether the behavior should become 
//   dominant. Behaviors take effect if they were previously dominant 
//   and not finished processing their action (ie, FSM is not complete) 
//   or conditions are detected that mean the behavior should become
//   dominant.
// 
// - doBehavior() to execute the FSM associated with the behavior.
//   On startup the FSM should set its name as the current behavior and
//   on completion reset the current behavior to the value in 
//   'defaultBehavior'.
//
// 

bool activateEscape(void)
// Check if the ESCAPE conditions are satisfied.
// Escape will always take priority over any other behavior selected.
{
  bool b = false;
  
  b = b || (runBehavior == ESCAPE);                       // currently dominant ...
  b = b || ((Sensors.getBumperAll() & BUMP_FRONT) != 0);  // ... or any front bumper triggered ...
  b = b || Sensors.getScan(cSensors::SC) < DIST_IMPACT;  // ... or too close on scan

  return(b);
}

void doEscape(bool restart)
// Escapes danger
// Implements an augmented algorithm based on the method described at
// http://schursastrophotography.com/robotics/bumperlogic.html
{
  // If MAX_EVENT escapes occur within EVENT_SPAN_TIME seconds, then we consider that
  // we are entrapped in a corner and may be 'canyoning'.
  // timeEvent[] stores the time for MAX_EVENT co0llisions so we can work out time 
  // difference between the first (timeEvent[0]) and the latest (timeEvent[MAX_EVENT-1])
  //
  const uint8_t MAX_EVENT = 4;          // max number of escapes tracked ...
  const uint32_t EVENT_TIME_SPAN = 20;  // ... in this time span (in seconds)
  static struct
  {
    uint32_t time;          // the millis() value
    uint8_t type;           // to detect a L/R pattern
  } timeEvent[MAX_EVENT];

  // When we get a collision, back up the wheels by 
  // REVERSE_ANGLE radians (2*PI = 1 rotation)
  const float REVERSE_ANGLE = PI/2;

  int8_t pctRotate;                   // percent adjustment rotation
  static uint16_t timeToScan;         // time to scan around
  static uint8_t bumperHit = 0;       // bumpers at the time of collision
  static enum { IDLE, BACKUP, DECIDE, BAILOUT, SPIN } mode = IDLE;
  static uint32_t timeMark;           // millis timer mark

  if (restart)
  {
    runBehavior = ESCAPE;
    Sensors.setScanPeriod(SCAN_FAST_POLL);
    mode = IDLE;

    // shuffle the event time in the array and later add the time 
    // for the latest event in the last position
    for (uint8_t i = 1; i < MAX_EVENT; i++)
      timeEvent[i - 1] = timeEvent[i];
  }

  switch (mode)
  {
  case IDLE:   // just hit something, so back up and look around 
    TEL_MESG("\nESCAPE");
    bumperHit = Sensors.getBumperAll();   // save for later analysis

    // if we are detecting 'too close' by scan, make it look like a front bumper hit
    if (Sensors.getScan(cSensors::SC) < DIST_IMPACT)
      bumperHit |= _BV(cSensors::FC);

    // set up the next event recording
    timeEvent[MAX_EVENT - 1].time = millis();
    if (bumperHit & BUMP_LEFT) timeEvent[MAX_EVENT - 1].type = BUMP_LEFT;
    else if (bumperHit & BUMP_RIGHT) timeEvent[MAX_EVENT - 1].type = BUMP_RIGHT;
    else timeEvent[MAX_EVENT - 1].type = BUMP_MID;

    // start scanning all around us while we are reversing
    Sensors.setScanMask(SCAN_ALL);
    timeToScan = Sensors.getSweepTime();

    // now move back
    Car.setLinearVelocity(SPEED_MOVE);
    Car.move(-REVERSE_ANGLE, -REVERSE_ANGLE);
    timeMark = millis();
    mode = BACKUP;
    break;

  case BACKUP:
    // if we are bumping at the back, stop the sequence now and 
    // move forward slightly, then continue as if we have finished.
    if (Sensors.getBumperAll() & BUMP_BACK)
    {
      TEL_MESG(" OOPS!");
      Car.stop();
      Car.run();    //make it do it now
    }

    // keep waiting until completed
    if (!Car.isRunning())
    {
      // if we have exceeded the number of events in EVENT_TIME_SPAN we 
      // could be canyoning, so bail out rather than trying to escape
      mode = DECIDE;    // default choice which we change below
      if (timeEvent[0].time != 0 && timeEvent[MAX_EVENT - 1].time - timeEvent[0].time < (EVENT_TIME_SPAN * 1000))
      {
        bool b = true;
        // see if this is an alternating hits type pattern
        for (uint8_t i = 0; b && i < MAX_EVENT - 1; i++)
          b = b && (timeEvent[i].type != timeEvent[i + 1].type);
        if (b) mode = BAILOUT;
      }
    }
    break;

  case DECIDE:
    // first up, give the scan data collection time to finish (if not already)
    // so that we have all the data for distances around us.
    if (Sensors.isScanEnabled() && (millis() - timeMark < timeToScan))
      break;

    // From here backup sequence is now complete and we should have sensed 
    // all distances (if sensor enabled), so based on how we originally hit 
    // the obstacle work out the turning amount to get out of this.
    
    // check if we hit the FC bumper as this is an important event
    if (bumperHit & (1 << cSensors::FC))
    {
      TEL_MESG(" FC");
      pctRotate = 13 + random(13);      // 13% turn (45 degrees) + random amount up to 25% (90 degrees)

      // Now check if we need to change this to a left turn
      // based on historical bump sequences.

      // if we hit center the last time as well, make it a always left turn
      // if we don't have scan to go by (checked later)
      if (!Sensors.isScanEnabled() && timeEvent[MAX_EVENT - 2].type == BUMP_MID)
      {
        TEL_MESG(" C=");
        pctRotate = -pctRotate;
      }
      // previous 2 bumps were right side, now bumped center, so rotate left
      else if (timeEvent[MAX_EVENT - 2].type == timeEvent[MAX_EVENT - 3].type
        && timeEvent[MAX_EVENT - 2].type != BUMP_MID)
      {
        TEL_MESG(" 2=");
        if (timeEvent[MAX_EVENT - 2].type == BUMP_RIGHT)
          pctRotate = -pctRotate;
      }
      else if (Sensors.getScan(cSensors::SL) > Sensors.getScan(cSensors::SR))
      {
        TEL_MESG(" >");
        // no pattern to the bumps, so pick the side with the most space to move
        pctRotate = -pctRotate;
      }
      else if (Sensors.getScan(cSensors::SL) == Sensors.getScan(cSensors::SR))
      {
        TEL_MESG(" RA");
        // randomly select whether to change to left rotation
        if (random(100) > 50) pctRotate = -pctRotate;
      }
    }
    else
    {
      // Sequence of side impact checks gives minimum amount of turn priority 
      // over more drastic turns. Also it just assume +ve turn (right) and
      // we'll adjust that later if needed.

      // if the edge sensor on far side was bumped, veer gently away from the bumped side
      if ((bumperHit & (1 << cSensors::FLL)) || (bumperHit & (1 << cSensors::FRR)))
      {
        // Front LL or RR sensor is bumped
        TEL_MESG(" FLL/RR");
        pctRotate = 4 + random(4);      // 4% (11 degrees) + random amount up to 7% (22 degrees)
      }
      // if the middle sensor either side was bumped, veer away from the bumped side
      else if ((bumperHit & (1 << cSensors::FL)) || (bumperHit & (1 << cSensors::FR)))
      {
        // Front L or R sensor is bumped
        TEL_MESG(" FL/R");
        pctRotate = 7 + random(7);      // 7% (22 degrees) + random amount up to 12% (45 degrees)
      }

      // if we are hit anywhere on the right hand side, we need to steer left instead
      if (bumperHit & BUMP_RIGHT) pctRotate = -pctRotate;
    }
    
    // All that is left are the back bumpers...
    // if we are hit anywhere on the back, this is unusual so do 
    // something creative and override any of the logic above
    if (bumperHit & BUMP_BACK)
    {
      TEL_MESG(" BL/C/R");
      pctRotate = 25 - random(50);     // head somewhere randomly (+/- 25% or 90 degrees)    
    }

    // display telemetry info on decision
    if (pctRotate < 0) TEL_MESG(" L");
    else if (pctRotate > 0) TEL_MESG(" R");
    TEL_VALUE(" %", pctRotate);

    // now actually tell the car to move
    Sensors.setScanMask(0);  // stop the scan scan
    Car.setLinearVelocity(SPEED_MOVE);
    Car.spin(pctRotate);
    mode = SPIN;
    break;

  case BAILOUT:
    // we have had too little time between MAX_EVENTS, so turn around and do
    // something different to avoid being stuck here forever.
    TEL_MESG(" BAIL");
    timeEvent[1].time = 0;     // clear this so we skip the next impact being detected as an entrapment
    Car.setLinearVelocity(SPEED_MOVE);
    Car.spin(50);         // turn 50% revolution (180 degrees)
    mode = SPIN;
    break;

  case SPIN:   // wait for the spin adjustment to stop
    if (!Car.isRunning())     // movement has completed
    {
      TEL_MESG(": end");
      runBehavior = defaultBehavior;
      mode = IDLE;
    }
    break;
  }
}

bool activateAvoid(void)
// Check if AVOID conditions are satisfied.
{
  bool b = false;

  b = b || (runBehavior == AVOID);         // currently dominant
  b = b || Sensors.getScan(cSensors::SC) < DIST_OBSTACLE;  // within range of obstruction
  b = b && (defaultBehavior == CRUISE);    // but only if this is the selected overall behavior

  return(b);
}

void doAvoid(bool restart)
// Avoids collision when in cruise mode.
// This veers in the direction of the most free space, with proportionately
// more rotation the closer vehicle is to the obstacle.
{
  static uint16_t timeToScan;         // time to scan around
  static enum { IDLE, DECIDE, WAIT } mode = IDLE;
  static uint32_t timeMark;           // millis timer mark

  if (restart)
  {
    Sensors.setScanPeriod(SCAN_FAST_POLL);
    runBehavior = AVOID;
    mode = IDLE;
  }

  switch (mode)
  {
  case IDLE:      // too close to something, start avoidance moves
    TEL_MESG("\nAVOID");

    // start scanning at front 3 positions for an escape route
    Sensors.setScanMask(SCAN_MID);
    timeToScan = Sensors.getSweepTime();  // how long the scan will take
    Car.setLinearVelocity(SPEED_CRAWL);   // slow right down while scanning
    timeMark = millis();
    mode = DECIDE;
    break;

  case DECIDE:    // decide how which direction to veer away
    {
      // When we need to avoid, the maximum angle to veer 
      // away is AVOID_ANGLE_MAX radians/sec (2*PI = 1 rotation)
      const float AVOID_ANGLE_MAX = PI / 2;

      // wait for a scan cycle to complete
      if (millis() - timeMark < timeToScan)
        break;

      float turn = 0.0;   // this MAY be changed below

      // get current set of readings
      uint8_t L = Sensors.getScan(cSensors::SL);
      uint8_t C = Sensors.getScan(cSensors::SC);
      uint8_t R = Sensors.getScan(cSensors::SR);
      Sensors.setScanMask(0);        // stop active scanning

      //TEL_VALUE(" C ", C);
      //TEL_VALUE(" L ", L);
      //TEL_VALUE(" R ", R);

      // how much to turn? 
      // work out angle inverse to distance from obstacle and keep it 
      // to a max AVOID_ANGLE /sec rotation (ie, less turn further out, 
      // more turn closer to obstacle)
      timeMark = 0;
      if (DIST_OBSTACLE > C)
      {
        turn = (float)C / (float)DIST_OBSTACLE;
        turn = (1.0 - turn) * AVOID_ANGLE_MAX;    // complementary proportional

        // which way to turn? Default is R (+) but change to L (-) if there 
        // is more space that side
        if (L > R) turn = -turn;

        TEL_MESG(" turn ");
        if (turn > 0) TEL_VALUE("R ", turn);
        else          TEL_VALUE("L ", -turn);

        timeMark = millis();      // set wait threshold
      }

      // finally, set the path to avoid obstacle
      Car.drive(SPEED_CRUISE, turn);
      mode = WAIT;
    }
    break;

  case WAIT:    // wait for the avoid sequence to complete
    if (millis() - timeMark >= AVOID_ACTIVE_TIME)
    {
      TEL_MESG(" :end");
      runBehavior = defaultBehavior;
      mode = IDLE;
    }
    break;
  }
}

bool activateWallFollow(void)
// Check if the WALLFOLLOW conditions are satisfied.
{
  bool b = false;

  b = b || (Sensors.isScanEnabled());       // we need scan to run this!
  b = b && (defaultBehavior == WALLFOLLOW);  // but only if this is the selected behavior 

  return(b);
}

void doWallFollow(bool restart)
// Follows wall at set distance
// Implements a PID algorithm to control the distance
{
#define PUSH_1MODE(m)    { mode = (m); nextMode = SEARCH; }
#define PUSH_2MODE(m, n) { mode = (m); nextMode = (n); }
#define POP_MODE         { mode = nextMode; nextMode = SEARCH; }

#define DEBUG_PID 0   // info about PID on or off for tuning K's

  const float kP = 0.011;
  const float kI = 0.0;
  const float kD = 0.004;

  static struct
  {
    float p, i, d;              // intermediate results for debug
    float err, errSum, errLast; // error values
  } pid;
  const uint32_t PID_TIME = 1000;   // in milliseconds
  const float MAX_TURN = PI / 3;    // in radians

  static enum { START, SEARCH, TURN, CONTROL_INIT, CONTROL, DELAY } nextMode = SEARCH, mode = START;
  static uint32_t timeMark;         // millis timer marker
  static bool longSLLFound;         // flag for long SLL (see below)

  // decision outcome variable for setting the trajectory
  uint8_t velocity = SPEED_MAX;     // may be changed if a decision made

  if (restart)
  {
    runBehavior = WALLFOLLOW;
    PUSH_1MODE(START);
    longSLLFound = false;
  }

  switch (mode)
  {
  case START:
    // set things up for FOLLOW
    TEL_MESG("\nFOLLOW start");

    Car.drive(velocity);

    // start scanning at front and left
    Sensors.setScanMask(SCAN_WALL);
    Sensors.setScanPeriod(SCAN_WALL_POLL);
    timeMark = millis();
    POP_MODE;
    break;

  case SEARCH:
    // Move forward looking either for a wall directly 
    // in front or one within range on the LEFT side
    if (Sensors.getScan(cSensors::SLL) <= (2 * DIST_WALLFOLLOW))
    {
      // the wall is close enough to the left side to start tracking it
      TEL_MESG("\nFollow wall");
      PUSH_1MODE(CONTROL_INIT);
    }
    else if (Sensors.getScan(cSensors::SC) <= DIST_WALLFOLLOW)
    {
      // something directly in front, turn left in front of it 
      // and then start tracking 
      TEL_VALUE("\nDetect Front ", Sensors.getScan(cSensors::SC));
      Car.spin(25);
      PUSH_2MODE(TURN, CONTROL_INIT);
    }
    else if (Sensors.getScan(cSensors::SLL) <= DIST_WALLDETECT && 
             !longSLLFound)
    {
      // something on the left, turn towards it and then move to
      // it and keep searching. longSSLLFound makes sure we only
      // do this once as, if there are multiple targets, the vehicle 
      // could keep giong around in circles.
      longSLLFound = true;
      TEL_VALUE("\nDetect Left ", Sensors.getScan(cSensors::SLL));
      Car.spin(-25);
      PUSH_2MODE(TURN, SEARCH);
    }
    break;

  case TURN:  // Currently in a spin turning into or away from the wall
    // wait until the turn is complete and then 
    // start the next mode in the stack
    if (!Car.isRunning())
    {
      POP_MODE;
      Car.drive(velocity);
    }
    break;

  case CONTROL_INIT:
    // essentially reset the PID
    pid.errLast = pid.err = pid.errSum = 0.0;
    mode = CONTROL;
    // fall through into control

  case CONTROL:    // Run the PID control
    timeMark = millis();    // set up for next iteration

    // Now decide which direction to steer
    // First check if we have an object in front of us. If so we need to
    // turn to the right (+ angle) as we are following wall on the left
    if (Sensors.getScan(cSensors::SC) <= DIST_WALLFOLLOW)
    {
      TEL_MESG("\nBlocked ");
      Car.spin(13);
      PUSH_2MODE(TURN, CONTROL_INIT);
    }
    else
    {
      // How much to steer? 
      // Do the PID calculation to work out how much we need 
      // to turn in the next control time period

      // Run the PID control calculation ...
      pid.errLast = pid.err;    // save previous error
      pid.err = DIST_WALLFOLLOW - Sensors.getScan(cSensors::SLL);
      pid.errSum += pid.err;
      pid.p = kP * pid.err;
      pid.i = kI * pid.errSum;
      pid.d = kD * (pid.errLast - pid.err);
      float turn = pid.p + pid.i + pid.d;

#if DEBUG_PID
      TEL_VALUE("\nE=", pid.err);
      TEL_VALUE(" PID=[", pid.p);
      TEL_VALUE(" ,", pid.i);
      TEL_VALUE(" ,", pid.d);
      TEL_MESG("]");
#endif
      // keep the number in sensible bounds
      if (abs(turn) > MAX_TURN)
      {
        turn = (turn < 0.0) ? -MAX_TURN : MAX_TURN;
#if DEBUG_PID
        TEL_MESG(" <|");
#endif
      }

      // The library needs an angular velocity (rad/s) to bring the LH 
      // side of the vehicle closer to DIST_WALLFOLLOW from the wall. 
      // Assume the angle calculated will be turned in PID_CONTROL time,
      // so this becomes the turning rate.
      turn *= ((float)PID_TIME / 1000.0);

      // Now enact the decisions made
      if (Car.getLinearVelocity() != velocity || Car.getAngularVelocity() != turn)
      {
#if !DEBUG_PID
        TEL_VALUE("\nFOLLOW LL:", Sensors.getScan(cSensors::SLL));
        TEL_MESG(" adj");
#endif
        TEL_VALUE(" r", turn);
        Car.drive(velocity, turn);  // set the path for wall distance
      }

      // Wait until the PID_TIME loop has expired, the go again
      PUSH_2MODE(DELAY, CONTROL);
    }
    break;

  case DELAY:   // wait for the completion of PID loop time
    if (millis() - timeMark >= PID_TIME)
      POP_MODE;
    break;
  }
}

void doCruise(bool restart)
// Default is to just drive in a straight line
// We only get here when all other behaviors are not applicable.
{
  if (restart)
  {
    TEL_MESG("\nCRUISE");
    Sensors.setScanPeriod(SCAN_SLOW_POLL);
    Sensors.setScanMask(SCAN_CTR);
    Car.drive(SPEED_CRUISE);
  }
  else
  {
    uint8_t v = Sensors.getScan(cSensors::SC) >= DIST_CLEAR ? SPEED_MAX : SPEED_CRUISE;

    if (Car.getLinearVelocity() != v)
    {
      DEBUG("\nCRUISE sensor ", Sensors.getScan(cSensors::SC));
      DEBUG(" speed ", v);
      Car.drive(v);
    }

#if SCDEBUG
    static uint16_t vL, vR;

    if (vL != ML.getSpeed() || vR != MR.getSpeed())
    {
      vL = ML.getSpeed();
      vR = MR.getSpeed();
      TEL_VALUE(" ML/R:", vL);
      TEL_VALUE("/", vR);
    }
#endif
  }
}

void setup(void)
{
#if SCDEBUG || ENABLE_DEBUG || DUMP_SENSORS
  Serial.begin(57600);
  DEBUGS("\n[SmartCar2 Debug Mode]\n");
#endif
  BTSerial.begin(BT_BAUDRATE);

  CP.begin();
  Buzzer.begin();
  if (!Sensors.begin())
    TEL_PANIC("Sensors start error");

  if (!Car.begin(PPR, PPS_MAX, DIA_WHEEL, LEN_BASE))   // take all the defaults
    TEL_PANIC("Unable to start car!!");
  Car.setLinearVelocity(0);    // vehicle is stopped
}

void loop(void)
{
  static uint32_t timerHealth;

  // ----------------------
  // Always run these background tasks
  CP.run();         // Command processor from BT or Serial Monitor
  Car.run();        // Car functions
  Buzzer.run();     // Buzzer functions
  Sensors.read();   // Read sensor data
  sendTelemetryData(Sensors.isUpdated() && runEnabled); // telemetry if changed or on internal timer when run not enabled
  // ----------------------

#if DUMP_SENSORS
  if (Sensors.isUpdated()) Sensors.dump(CMDStream);     // debug dump
#endif

  if (runEnabled)
  {
    // check health status every HEALTHCHECK_PERIOD
    if (millis() - timerHealth >= HEALTHCHECK_PERIOD)
    {
      if (!checkHealth()) stopVehicle();
      timerHealth = millis();
    }

    // Arbitrate behaviors in priority order
    if      (activateEscape())     { doEscape(runBehavior != ESCAPE); }
    else if (activateAvoid())      { doAvoid(runBehavior != AVOID); }
    else if (activateWallFollow()) { doWallFollow(restart); }
    else                           { doCruise(restart); }    // default choice

    // if the curreent behavior has changed, this
    // signals a restart for the next behavior
    restart = (runBehavior != prevBehavior);
    prevBehavior = runBehavior;   // save this for next time
  }
}
