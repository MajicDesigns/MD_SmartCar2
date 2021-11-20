#pragma once
/**
 * \file
 * \brief Header file for MD_SmartCar2 library hardware related parameters.
 */

/**
\page pageVehicleHardware Vehicle Hardware

![SmartCar2 Platform] (SmartCar2_Platform.png "SmartCar2 Platform")

The vehicle mechanical platform is modeled on similar commercially available 
models, but was redesigned in Fusion360 as a custom platform. The acrylic was 
cut on a laser printer and the brass hex standoff pillars are standard dimensions.

![SmartCar2 Decks] (SmartCar2_Decks.png "SmartCar2 Decks")

All files for this design, including the Fusion360 model, are available on 
Thingiverse at https://www.thingiverse.com/majicdesigns/designs.

### Motors and Wheels

The new platform is designed to work with 28BYJ-48 type stepper motors using 
custom brackets, shown below.

![SmartCar2 Motor Assembly] (SmartCar2_28BYJ-48_Assy_Front.png "SmartCar2 Motor Assembly")

The wheels were designed to fit the 28BYJ-48 motors. The outside rim was covered in 5mm 
self-adhesive foam to create a gripping tyre-like surface.

![SmartCar2 Wheel] (SmartCar2_Wheel.png "SmartCar2 Wheel")

Together these create an easy and securely connected drive block.

![SmartCar2 Motor Insitu] (SmartCar2_Motor_Insitu.jpg "SmartCar2 Motor In Use")

\page pageMotorController Motor Controller

The library manages the motor functions using the MD_Stepper library 
(https://github.com/MajicDesigns/MD_Stepper or the Arduino library manager), 
so any controllers supported by that library can be used.

MD_SmartCar2 was developed using inexpensive 28BYJ-48 stepper motors (12V version) 
bundled with a ULN2003 'stepper driver' board (high voltage, high current darlington 
transistor IC) that need 4 digital I/O pins to drive the motor (shown below).

![SmartCar2 Stepper] (SmartCar2_Stepper.png "SmartCar2 Stepper Motor")

\page pageHardwareMap Hardware Allocation Map  (Arduino Nano)

The library code is mostly independent of processor used, however the original 
target platform is an Arduino Nano. The pins listed here and defined in the 
SC2_HardwareDefs.h header file are reserved (R) by the library. Spare pins are 
for use by any application Application (A) built on top of this library.

| Pin  |Use| Description
|------|:--|:---------------
| D0*  | R | Hardware Serial Tx
| D1*  | R | Hardware Serial Rx
| D2!  | A | Spare
| D3!~ | A | Spare
| D4   | A | Spare
| D5~  | A | Spare
| D6~  | A | Spare
| D7   | A | Spare
| D8   | A | Spare
| D9~  | A | Spare
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

