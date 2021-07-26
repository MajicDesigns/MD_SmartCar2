// Test MD_SmartCar2 class through a Bluetooth interface to 
// an AI2 control application using a HC-05 BT module that has 
// been pre-initialized and paired to the BT master.
//
// This is to test 'real-world' motion after the vehicle 
// parameters have been calibrated.
// 
// All Vehicle motion types can be exercised and parameters fine
// tuned and saved to EEPROM from the AI2 'SmartCar2_Control-Setup' 
// interface application.
//
// SmartCar_HW.h contains all the hardware connection pin definitions.

#include <SoftwareSerial.h>
#include <MD_SmartCar2.h>
#include <MD_cmdProcessor.h>
#include "SmartCar2_HW.h"

#define ECHO_COMMAND 0    // Echo commands to the Serial stream for debugging

// Global Variables
// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper with 28BYJ-48
AccelStepper MR(AccelStepper::HALF4WIRE, PIN_INA1, PIN_INA3, PIN_INA2, PIN_INA4);
AccelStepper ML(AccelStepper::HALF4WIRE, PIN_INB2, PIN_INB4, PIN_INB1, PIN_INB3);

MD_SmartCar2 Car(&ML, &MR);  // SmartCar object

const uint8_t FP_SIG = 2;   // floating point significant decimal points

// Global Objects and Variables
SoftwareSerial BTSerial(PIN_BT_RX, PIN_BT_TX);

// handler function prototypes
void handlerD(char* param);
void handlerM(char* param);
void handlerS(char* param);

const MD_cmdProcessor::cmdItem_t PROGMEM cmdTable[] =
{
  { "d",  handlerD,  "v a",     "Drive lin vel v [-100..100] ang vel a [-90..90]", 2 },
  { "m",  handlerM,  "l r",     "Move wheels subtended angle l, r", 2 },
  { "s",  handlerS,  "f",       "Spin full circle fraction f% [-100, 100]", 2 },
};

MD_cmdProcessor CP(BTSerial, cmdTable, ARRAY_SIZE(cmdTable));

// handler functions
void handlerD(char* param)
{
  int v, a;

#if ECHO_COMMAND
  Serial.println(CP.getLastCmdLine());
#endif
  sscanf(param, "%d %d", &v, &a);
  //v = trunc(fv);
  //a = trunc(fa);
  if (v < -100) v = -100;
  if (v > 100) v = 100;
  if (a < -90) a = -90;
  if (a > 90) a = 90;

  Car.drive((int8_t)v, (int8_t)a);
}

void handlerM(char* param)
{
  int al, ar;

#if ECHO_COMMAND
  Serial.println(CP.getLastCmdLine());
#endif
  sscanf(param, "%d %d", &al, &ar);

  Car.move((int16_t)al, (int16_t)ar);
}

void handlerS(char* param)
{
  int f;

#if ECHO_COMMAND
  Serial.println(CP.getLastCmdLine());
#endif
  sscanf(param, "%d", &f);

  Car.spin((int16_t)f);
}
 
void setup(void)
{
#if ECHO_COMMAND || SCDEBUG || PID_TUNE
  Serial.begin(57600);
#endif
  BTSerial.begin(BT_BAUDRATE);
  if (!Car.begin(PPR, PPS_MAX, DIA_WHEEL, LEN_BASE))   // take all the defaults
    BTSerial.print(F("\n\n!! Unable to start car"));

  CP.begin();
}

void loop(void)
{
  Car.run();
  CP.run();
}
