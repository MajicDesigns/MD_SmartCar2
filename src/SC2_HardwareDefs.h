#pragma once
/**
 * \file
 * \brief Header file for MD_SmartCar library hardware related parameters.
 */

/**
\page pageHardwareMap Hardware Allocation Map 

## Hardware Functional Allocation for Arduino Nano

The library code is mostly independent of processor used, however the original 
target platform is an Arduino Nano. The pins listed here and defined in the 
SC_HardwareDefs.h header file are reserved (R) by the library. Spare pins are 
for use by any application Application (A) built on top of this library.

| Pin  |Use| Description
|------|:--|:---------------
| D0*  | R | Hardware Serial Tx
| D1*  | R | Hardware Serial Rx
| D2!  | R | Spare
| D3!~ | R | Spare
| D4   | R | Spare
| D5~  | R | Spare
| D6~  | R | Spare
| D7   | R | Spare
| D8   | R | Spare
| D9~  | R | Spare
| D10~ | A | Spare / Default SPI SS
| D11~*| A | Spare / Hardware SPI MOSI
| D12* | A | Spare / Hardware SPI MISO
| D13* | A | Spare / Hardware SPI SCK
| A0   | A | Spare
| A1   | A | Spare
| A2   | A | Spare
| A3   | A | Spare 
| A4*  | A | Spare / Hardware I2C SDA
| A5*  | A | Spare / Hardware I2C SCL
| A6   | A | Spare / Nano Analog I/O only
| A7   | A | Spare / Nano Analog I/O only
 
(*) shared or comms bus pins, (~) hardware PWM pin, (!) external iRQ pin
*/

