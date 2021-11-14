#pragma once

// SmartCar2 application hardware, timing and other definitions

// ------------------------------------
// SmartCar2 Physical Constants
const uint16_t PPR = 4038;      ///< Stepper pulses per revolution
const uint16_t PPS_MAX = 1000;  ///< Stepper maximum speed in pulses per second
const uint16_t DIA_WHEEL = 65;  ///< Wheel diameter in mm
const uint16_t LEN_BASE = 110;  ///< Wheel base in mm (= distance between wheel centers)

const uint8_t SPEED_CRAWL = 25;  ///< default speed for low speed crawl (% full speed)
const uint8_t SPEED_MOVE = 50;   ///< default speed for moves (% full speed)
const uint8_t SPEED_CRUISE = 80; ///< cruising speed for vehicle (% full speed)
const uint8_t SPEED_MAX = 100;   ///< maximum speed for vehicle (% full speed) 

// ------------------------------------
// Motor Controller pins
// 
// Right Motor
const uint8_t PIN_INA1 = 2;    ///< Motor A Mode pin 1 - simple digital pin
const uint8_t PIN_INA2 = 3;    ///< Motor A Mode pin 2 - simple digital pin
const uint8_t PIN_INA3 = 4;    ///< Motor A Mode pin 3 - simple digital pin
const uint8_t PIN_INA4 = 5;    ///< Motor A Mode pin 4 - simple digital pin
// Left Motor
const uint8_t PIN_INB1 = 11;   ///< Motor B Mode pin 1 - simple digital pin
const uint8_t PIN_INB2 = 10;   ///< Motor B Mode pin 2 - simple digital pin
const uint8_t PIN_INB3 = 7;    ///< Motor B Mode pin 3 - simple digital pin
const uint8_t PIN_INB4 = 6;    ///< Motor B Mode pin 3 - simple digital pin

// ------------------------------------
// Bluetooth connections using SoftwareSerial
const uint8_t PIN_BT_RX = 8;   ///< Arduino RX, connect to to BT TX pin. Fixed pin usage for AltSoftSerial.
const uint8_t PIN_BT_TX = 9;   ///< Arduino TX, connect to to BT RX pin. Fixed pin usage for AltSoftSerial.

const uint16_t BT_BAUDRATE = 9600; ///< BT serial connection speed (bps)

// ------------------------------------
// Bumper Switches
// Uses hardware I2C which is A4, A5
const uint16_t BUMPER_POLL_PERIOD = 20;   ///< in milliseconds
const uint8_t PCF8574_ADDR = 0x20;        ///< PCF 8574 external I/O I2C address

// ------------------------------------
// Battery Voltage Sensor
const uint32_t V_POLL_PERIOD = 30000; ///< in milliseconds
const uint8_t PIN_VOLTAGE = A0;       ///< Pin connection for voltage divider output
const float V_FULL = 11.5;            ///< Battery full voltage for 100% scale

// ------------------------------------
// Sonar sensors connections (NewPing library - single pin mode)
const uint16_t SONAR_FAST_POLL = 300;   ///< in milliseconds
const uint16_t SONAR_SLOW_POLL = 1000;  ///< in milliseconds
const uint8_t PIN_SERVO = A3;  ///< Servo pin
const uint8_t PIN_SONAR = A2;  ///< Sonar ping sensor pin

// Define SONAR distance points in cm for decision making
const uint8_t DIST_ALLCLEAR = 255;  ///< All clear distance (more than DIST_MAX)
const uint8_t DIST_MAX = 200;       ///< Maximum distance to ping (for ping library)
const uint8_t DIST_IMPACT = 15;     ///< impact imminent
const uint8_t DIST_CLOSE = 30;      ///< really close 
const uint8_t DIST_OBSTACLE = 50;   ///< far obstacle detected
const uint8_t DIST_CLEAR = 60;      ///< distant object

// ------------------------------------
// Miscellaneous values
const uint32_t TELEMETRY_PERIOD = 1200;   ///< telemetry packet send period in ms when no data changed
const uint32_t AVOID_ACTIVE_TIME = 1500;  ///< time for AVOID to be acive in ms
const uint32_t FOLLOW_ACTIVE_TIME = 1000; ///< time for WALLFOLLOWER to be active in ms

const uint8_t FLOAT_DECIMALS = 2;         ///< default decimals shown in float values