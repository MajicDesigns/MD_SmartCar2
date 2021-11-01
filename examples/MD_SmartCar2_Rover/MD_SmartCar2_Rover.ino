// Application to control an autonomous MD_SmartCar2 vehicle using a 
// Bluetooth interface implemented with a HC-05 module that has been 
// pre-initialized and paired to the BT master.
// 
// This is an example of behaviors built on top of the library infrastructure.
// High level control of the robotic vehicle is modeled after 'Behavior Based Robotics'
//
// All vehicle behavior types can be exercised and monitored 
// from the AI2 'SmartCar2_Rover_Control' interface application.
//
// SmartCar2_HW.h contains all the hardware I/O pin definitions.
// 
// External library dependencies:
// - PCF8574 library available from https://github.com/RobTillaart/PCF8574.git or the IDE Library Manager
// 
// - AltSoftSerial is available from https://github.com/PaulStoffregen/AltSoftSerial or the IDE Library Manager
// Note: AltSoftSerial is hard coded to use digital pins 8 and 9 on the Arduino Nano.
// 
// - NewPing library available from https://bitbucket.org/teckel12/arduino-new-ping/src/master/ or the IDE Library Manager
// Note: TIMER_ENABLED must be turned off for the NewPing library
// 
// - ServoTimer2 library is available from https://github.com/nabontra/ServoTimer2
// Note: In ServoTimer2.h change MIN_PULSE_WIDTH to 500 and MAX_PULSE_WIDTH to 2500. 
//       This enables 0 to 180 degree swing in the servo.
//

#include <AltSoftSerial.h>
#include <MD_SmartCar2.h>
#include <MD_cmdProcessor.h>
#include "SmartCar2_Sensors.h"

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

// BT monitoring telemetry
#if ENABLE_TELEMETRY
#if ENABLE_DEBUG
#define TELStream Serial
#else // not debug
#define TELStream BTSerial
#endif
#define TEL_VALUE(s,v) do { TELStream.print(F(s)); TELStream.print(v); } while (false)
#define TEL_MESG(s)    do { TELStream.print(F(s)); } while (false)
#if ENABLE_DEBUG
#define TEL_PACKET(s)
#else // not debug
#define TEL_PACKET(s)  do { TELStream.print('$'); TELStream.print(s); TELStream.print('~'); } while (false)
#endif
#else // not telemetry
#define TEL_VALUE(s,v)
#define TEL_MESG(s)
#define TEL_PACKET(s)
#endif

// ------------------------------------
// Global states
bool runEnabled = (REMOTE_START == 0);

static enum 
{ 
  ESCAPE = 0,     // emergency bumping into things or way too close. Always gets priority.
  AVOID = 1,      // CRUISE selected, moves to best open space when it gets too close to obstacle.
  WALLFOLLOW = 2, // follow a wall.
  CRUISE = 3      // default operating when nothing else is running - move straight.
} defaultBehavior = CRUISE, runBehavior = CRUISE;

// ------------------------------------
// Global Variables
// Initialize with pin sequence IN1-IN2-IN3-IN4 for using the MD_Stepper with 28BYJ-48.
// Motors should be wired identically and the change of direction happens in pin order below.
MD_Stepper MR(PIN_INA1, PIN_INA2, PIN_INA3, PIN_INA4);  // rotate in fwd direction and ...
MD_Stepper ML(PIN_INB2, PIN_INB1, PIN_INB4, PIN_INB3);  // ... reverse (opposite) direction

MD_SmartCar2 Car(&ML, &MR);                     // SmartCar2 control object
AltSoftSerial BTSerial(PIN_BT_RX, PIN_BT_TX);   // BT comms link

// ------------------------------------
// Sonar scanning groups
// These are bit masks that define related sets of sonar scanning positions in
// different related sets that we may want to use.
//
const uint8_t SCAN_MID = (_BV(cSensors::SL) | _BV(cSensors::SC) | _BV(cSensors::SR));
const uint8_t SCAN_CTR = _BV(cSensors::SC);
const uint8_t SCAN_LEFT  = (_BV(cSensors::SLL) | _BV(cSensors::SL) | _BV(cSensors::SC));
const uint8_t SCAN_RIGHT = (_BV(cSensors::SC) | _BV(cSensors::SR) | _BV(cSensors::SRR));
const uint8_t SCAN_ALL = (SCAN_LEFT | SCAN_MID | SCAN_RIGHT);

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

void handlerM(char* param)
// Change running mode
{
  DEBUGS("Mode ");
  switch (*param)
  {
  case '1': runBehavior = WALLFOLLOW;              DEBUGS("WALL\n");   break;
  default:  runBehavior = CRUISE;                  DEBUGS("CRUISE\n"); break;
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
    Car.setLinearVelocity(SPEED_CRUISE);
    TEL_MESG("\n>> RUN <<");
  }
  else
  {
    Car.stop();
    Sensors.setSonarMask(SCAN_CTR);
    runBehavior = defaultBehavior;
    TEL_MESG("\n>> STOP <<");
  }
}

void handlerS(char* param)
// Run on or off
{
  bool enabled = (*param == '1');

  DEBUG("Sonar ", enabled);  DEBUGS("\n");
  Sensors.enableSonar(enabled);
}

void handlerH(char* param);   // prototype for cmdTable

const MD_cmdProcessor::cmdItem_t PROGMEM cmdTable[] =
{
  { "?", handlerH,  "", "Help text", 1 },
  { "r", handlerR, "b", "Run 0=stop, 1=start", 1 },
  { "m", handlerM, "m", "Mode 0=Cruise, 1=Wall, 2=Light, 3=Dark", 1 },
  { "s", handlerS, "b", "Sonar 0=disable, 1=enable" },
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
// | 07| AAAAA | Angular velocity in radians (3.2 float signed)
// | 12| BBB   | Bumper bitfield (decimal No) MSB [FLL|FL|FC|FR|FRR|BL|BC|BR] LSB
// | 15| FLL   | Sonar FLL distance (cm)
// | 18| FL_   | Sonar FL distance (cm)
// | 21| FC_   | Sonar FC distance (cm)
// | 24| FR_   | Sonar FR distance (cm)
// | 27| FRR   | Sonar FRR distance (cm)
// | 30| S     | Packet Sequence number (0-9)
// | 31|       | End of packet
//
{
#if ENABLE_TELEMETRY
  static uint32_t timeLast = 0;
  static int8_t seqNo = 0;

  if (newData || (millis() - timeLast >= TELEMETRY_PERIOD))
  {
    static char mesg[] = { "SRDCVVVVAAAAABBBFLLFL_FC_FR_FRR" };
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

    p += num2ASCII(p, Sensors.getSonar(cSensors::SLL), 3);
    p += num2ASCII(p, Sensors.getSonar(cSensors::SL), 3);
    p += num2ASCII(p, Sensors.getSonar(cSensors::SC), 3);
    p += num2ASCII(p, Sensors.getSonar(cSensors::SR), 3);
    p += num2ASCII(p, Sensors.getSonar(cSensors::SRR), 3);

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
  b = b || Sensors.getSonar(cSensors::SC) < DIST_IMPACT;  // ... or too close on sonar

  return(b);
}

void doEscape(bool restart)
// Escapes danger
// Backs away, turns to and resumes default behavior.
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
    mode = IDLE;

    // shuffle the event time in the array and later add the time 
    // for the latest event in the last position
    for (uint8_t i = 1; i < MAX_EVENT; i++)
      timeEvent[i - 1] = timeEvent[i];
  }

  switch (mode)
  {
  case IDLE:   // just hit something, so back up and look around 
    TEL_MESG("\nESCAPE start");
    bumperHit = Sensors.getBumperAll();   // save for later analysis

    // if we are detecting 'too close' by sonar, make it look like a front bumper hit
    if (Sensors.getSonar(cSensors::SC) < DIST_IMPACT)
      bumperHit |= _BV(cSensors::FC);

    // set up the next event recording
    timeEvent[MAX_EVENT - 1].time = millis();
    if (bumperHit & BUMP_LEFT) timeEvent[MAX_EVENT - 1].type = BUMP_LEFT;
    else if (bumperHit & BUMP_RIGHT) timeEvent[MAX_EVENT - 1].type = BUMP_RIGHT;
    else timeEvent[MAX_EVENT - 1].type = BUMP_MID;

    // start scanning all around us while we are reversing
    Sensors.setSonarMask(SCAN_ALL);
    timeToScan = Sensors.getSonarScanTime();

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
    // first up, wait for the sonar data collection time to finish (if not already)
    // so that we have all the data for distances around us.
    if (Sensors.isSonarEnabled() && (millis() - timeMark < timeToScan))
      break;

    Sensors.setSonarMask(0);  // stop the full sonar scan

    // From here backup sequence is now complete and we should have sensed 
    // all distances (if enabled), so based on how we originally hit the obstacle
    // work out the turning amount to get out of this.
    // The sequence of checks gives the minimum amount of turn priority over more
    // drastic turns.
    
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
    else if (bumperHit & (1 << cSensors::FC))
    {
      TEL_MESG(" FC");
      pctRotate = 13 + random(13);      // 13% (45 degrees) + random amount up to 25% (90 degrees)

      // Now we are set up for a right turn, so check if we need to change this
      // to a left one (change sign for turn percentage).

      // if we hit center the last time as well, make it a always left turn
      // if we don't have sonar to go by (later)
      if (!Sensors.isSonarEnabled() && timeEvent[MAX_EVENT - 2].type == BUMP_MID)
      {
        TEL_MESG(" ^");
        pctRotate = -pctRotate;
      }
      // if last 2 bumps were right side, now bumped center, so rotate left
      else if (timeEvent[MAX_EVENT - 2].type == timeEvent[MAX_EVENT - 3].type 
            && timeEvent[MAX_EVENT - 2].type != BUMP_MID)
      {
        TEL_MESG(" @");
        if (timeEvent[MAX_EVENT - 2].type == BUMP_RIGHT)
          pctRotate = -pctRotate;
      }
      else if (Sensors.getSonar(cSensors::SL) > Sensors.getSonar(cSensors::SR))
      {
        TEL_MESG(" >");
        // no pattern to the bumps, so pick the side with the most space to move
        pctRotate = -pctRotate;
      }
      else if (Sensors.getSonar(cSensors::SL) == Sensors.getSonar(cSensors::SR))
      {
        TEL_MESG(" !");
        // randomly select whether to change to left rotation
        if (random(100) > 50) pctRotate = -pctRotate;
      }
    }
    else
    {
      // All that is left are the back bumpers...
      TEL_MESG(" BL/C/R");
      pctRotate = 25 - random(50);     // head somewhere randomly (+/- 25% or 90 degrees)    
    }

    // if we are hit anywhere on the right hand side, we need to veer left
    if (bumperHit & BUMP_RIGHT) pctRotate = -pctRotate;
    
    // display telemetry info on decision
    if (pctRotate < 0)   TEL_MESG(" L ");
    else if (pctRotate > 0) TEL_MESG(" R ");
    TEL_VALUE("%", pctRotate);

    // now actually tell the car to move
    Car.setLinearVelocity(SPEED_MOVE);
    Car.spin(pctRotate);
    mode = SPIN;
    break;

  case BAILOUT:
    // we have had too little time between MAX_EVENTS, so turn around and do
    // something different to avoid being stuck here forever.
    TEL_MESG(" BAIL");
    timeEvent[1].time = 0;     // clear this so we skip the next impact as a potential entrapment situation
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
  b = b || Sensors.getSonar(cSensors::SC) < DIST_OBSTACLE;  // within range of obstruction
  b = b && (defaultBehavior == CRUISE);    // but only if this is the selected overall behavior

  return(b);
}

void doAvoid(bool restart)
// Avoids collision when in cruise mode.
// This veers in the direction of the most free space, 
// more veer with closer distance to obstacle.
{
  // When we need to avoid, the maximum angle that we want 
  // to veer away is AVOID_ANGLE radians/sec (2*PI = 1 rotation)
  const float AVOID_ANGLE = PI / 4;

  static uint16_t timeToScan;         // time to scan around
  static enum { IDLE, DECIDE, WAIT } mode = IDLE;
  static uint32_t timeMark;           // millis timer mark

  if (restart)
  {
    runBehavior = AVOID;
    mode = IDLE;
  }

  switch (mode)
  {
  case IDLE:      // too close to something, start avoidance moves
    TEL_MESG("\nAVOID start");

    // start scanning at front for an escape route
    Sensors.setSonarMask(SCAN_MID);
    timeToScan = Sensors.getSonarScanTime();  // how long the scan will take
    Car.setLinearVelocity(SPEED_CRAWL);       // slow right down while scanning
    timeMark = millis();
    mode = DECIDE;
    break;

  case DECIDE:    // decide how which direction to veer away
    {
      // wait for a scan cycle to complete
      if (millis() - timeMark < timeToScan)
        break;

      // how much to turn? 
      // work out angle inverse to distance from obstacle and keep it 
      // to a max AVOID_ANGLE /sec rotation (ie, less turn further out, 
      // more turn closer to obstacle)
      float turn = (1.0 - ((float)Sensors.getSonar(cSensors::SC) / (float)DIST_OBSTACLE)) * AVOID_ANGLE;

      Sensors.setSonarMask(0); // stop active scanning

      // which way to turn? Default is R (+) but change
      // to L (-) if there is more space that side
      if (Sensors.getSonar(cSensors::SL) > Sensors.getSonar(cSensors::SR))
        turn = -turn;
      if (turn < 0) TEL_MESG(": L"); else TEL_MESG(": R");
      TEL_VALUE(" ", turn);

      // Set the path to avoid obstacle
      Car.drive(SPEED_CRUISE, turn);
      timeMark = millis();
      mode = WAIT;
    }
    break;

  case WAIT:    // wait for the avoid sequence to complete
    if (millis() - timeMark >= AVOID_ACTIVE_TIME)
    {
      TEL_MESG(": end");
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

  // CURRENTLY NOT IMPLEMENTED
  //b = b || (runBehavior == WALLFOLLOW);      // currently dominant
  //b = b && (defaultBehavior == WALLFOLLOW);  // but only if this is the selected behavior 

  return(b);
}

void doWallFollow(bool restart)
// Follows wall at set distance
{
  static MD_SmartCar2::actionItem_t seqFollow[] =
  {
    { MD_SmartCar2::DRIVE, SPEED_CRUISE, 0 },       // angular filled in at run time
    { MD_SmartCar2::PAUSE, FOLLOW_ACTIVE_TIME }, // drive curved for a short time
    { MD_SmartCar2::END }
  };

  if (restart)
  {
    runBehavior = WALLFOLLOW;

    TEL_MESG("\nFOLLOW start");
    Car.startSequence(seqFollow);
  }
  else if (Car.isSequenceComplete())
  {
    TEL_MESG(": end");
    runBehavior = defaultBehavior;
  }
}

bool doCruise(bool restart)
// Default is to just drive in a straight line
// We only get here when all other behaviors are not applicable.
{
  if (restart)
  {
    TEL_MESG("\nCRUISE start");
    Sensors.setSonarMask(0);
    Car.drive(SPEED_CRUISE);
  }
  else
  {
    uint8_t v = Sensors.getSonar(cSensors::SC) >= DIST_CLEAR ? SPEED_MAX : SPEED_CRUISE;

    if (Car.getLinearVelocity() != v)
    {
      DEBUG("\nCRUISE sensor ", Sensors.getSonar(cSensors::SC));
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

  return(false);
}

void setup(void)
{
#if SCDEBUG || ENABLE_DEBUG || DUMP_SENSORS
  Serial.begin(57600);
  DEBUGS("\n[SmartCar2 Debug Mode]\n");
#endif
  BTSerial.begin(BT_BAUDRATE);

  CP.begin();
  Sensors.begin();
  if (!Car.begin(PPR, PPS_MAX, DIA_WHEEL, LEN_BASE))   // take all the defaults
    TEL_MESG("\nUnable to start car!!\n");
  Car.setLinearVelocity(0);    // something as a default
}

void loop(void)
{
  static bool restart = true;

  // Always run these background tasks
  CP.run();         // Command processor from BT or Serial Monitor
  Car.run();        // Car functions

  // Read sensors and output as appropriate
  Sensors.read();   // read sensors
#if DUMP_SENSORS
  if (Sensors.isUpdated()) Sensors.dump(CMDStream);     // debug dump
#endif
  sendTelemetryData(Sensors.isUpdated() && runEnabled); // telemetry if changed or on internal timer when run not enabled

  // If the global running flag is on arbitrate behaviors in priority order
  if (runEnabled)
  {
    if (activateEscape()) { restart = true; doEscape(runBehavior != ESCAPE); }
    else if (activateAvoid()) { restart = true; doAvoid(runBehavior != AVOID); }
    else if (activateWallFollow()) { restart = true; doWallFollow(runBehavior != WALLFOLLOW); }
    else { restart = doCruise(restart); }    // default choice
  }
}
