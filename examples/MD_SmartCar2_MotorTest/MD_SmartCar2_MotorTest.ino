// Test MD_SMartCar2 Motors functions
//
// This sketch is used to test and set up the stepper motors
// - AccelStepper setup L/R mnotor independently
// - Drive() control logic
// - Move() control logic
//
// Application is controlled from the Serial interface.
// Set Serial monitor to 57600 and ensure line ending is 'newline'
//

#include <MD_SmartCar2.h>
#include <MD_cmdProcessor.h>
#include "SmartCar2_HW.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#endif

// Global Variables
// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper with 28BYJ-48
AccelStepper MR(AccelStepper::HALF4WIRE, PIN_INA1, PIN_INA3, PIN_INA2, PIN_INA4);
AccelStepper ML(AccelStepper::HALF4WIRE, PIN_INB2, PIN_INB4, PIN_INB1, PIN_INB3);

MD_SmartCar2 SC(&ML, &MR);

bool modeStp;     // true if we are in AccelStepper testing mode

// cmdProcessor handlers and support functions 
void handlerHelp(char* param);

// INDIVIDUAL MOTORS TESTING
void motorSpeed(AccelStepper &M, char m, char* param)
{
  int16_t s = 0;

  Serial.print(F("\nAS> Speed "));
  Serial.print(m);
  Serial.print(F(" "));
  s = strtoul(param, nullptr, 10);
  Serial.print(s);
  Serial.print(F("% : "));
  M.setSpeed((s * M.maxSpeed())/100);
  Serial.print(M.speed());
  modeStp = true;
}

void setSpeed(int16_t s)
{
  ML.setMaxSpeed(s);
  MR.setMaxSpeed(s);
  ML.setAcceleration(s / 2);
  MR.setAcceleration(s / 2);
  modeStp = true;
}

void handlerSL(char* param) { motorSpeed(ML, 'L', param); }
void handlerSR(char* param) { motorSpeed(MR, 'R', param); }

void handlerSM(char* param)
{
  int16_t s = 0;

  Serial.print(F("\nAS> MaxSpeed "));
  s = strtoul(param, nullptr, 10);
  setSpeed(s);
  Serial.print(ML.maxSpeed());
  modeStp = true;
}

void handlerXX(char* param) { ML.setSpeed(0); MR.setSpeed(0); modeStp = true; }

// LIBRARY FUNCTIONS TESTING
void handlerD(char* param)
{
  int v, a;

  Serial.print(F("\nSC2> Drive "));
  sscanf(param, "%d %d", &v, &a);
  if (v < -100) v = -100;
  if (v > 100) v = 100;
  if (a < -90) a = -90;
  if (a > 90) a = 90;

  SC.drive((int8_t)v, (int8_t)a);

  Serial.print(SC.getLinearVelocity());
  Serial.print(F(" "));
  Serial.print(SC.getAngularVelocity());
  modeStp = false;
}

void handlerM(char* param)
{
  int al, ar;

  Serial.print(F("\nSC2> Move "));
  sscanf(param, "%d %d", &al, &ar);
  Serial.print(al);
  Serial.print(F(" "));
  Serial.print(ar);
  SC.move((int16_t)al, (int16_t)ar);
  modeStp = false;
}

void handlerS(char* param)
{
  int f;

  Serial.print(F("\nSC2> Spin "));
  sscanf(param, "%d", &f);
  Serial.print(f);
  SC.spin((int16_t)f);
  modeStp = false;
}

void handlerX(char* param) { SC.stop(); modeStp = false; }

const MD_cmdProcessor::cmdItem_t PROGMEM cmdTable[] =
{
  { "?",  handlerHelp, "",   "Help", 0 },
  { "h",  handlerHelp, "",   "Help", 0 },
  { "sl", handlerSL,   "n",  "AS Left speed setting to n % FS [+/- 100]", 1 },
  { "sr", handlerSR,   "n",  "AS Right speed setting to n % FS [+/- 100]", 1 },
  { "sm", handlerSM,   "m",  "AS Speed Max in pulse/sec", 1 },
  { "xx", handlerXX,    "",  "AS Stop all motors", 1 },
  { "d",  handlerD,  "v a",  "SC2 Drive vel v [-100..100] ang a [-90..90]", 2 },
  { "m",  handlerM,  "l r",  "SC2 Move wheels subtended angle l, r degrees", 2},
  { "s",  handlerS,    "f",  "SC2 Spin full circle fraction f% [-100, 100]", 2 },
  { "x",  handlerX,    "",   "SC2 stop motors", 2}
};

MD_cmdProcessor CP(Serial, cmdTable, ARRAY_SIZE(cmdTable));

void handlerHelp(char* param)
{
  Serial.print(F("\nHelp\n----"));
  CP.help();
}

void setup(void)
{
  Serial.begin(57600);
  Serial.print(F("\nMD_SmartCar2 - Stepper Motor Tester\n-----------------------------------"));
  Serial.print(F("\nEnter command. Ensure line ending set to newline.\n"));

  // initialize SmartCar2 stuff
  SC.begin(PPR, PPS_MAX, DIA_WHEEL, LEN_BASE);

  // start command processor
  CP.begin();
  CP.help();
}

void loop(void)
{
  CP.run();
  if (modeStp)
  {
    ML.runSpeed();
    MR.runSpeed();
  }
  else
    SC.run();
}
