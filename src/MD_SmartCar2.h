#pragma once
/**
\mainpage Robotic Vehicle Library

This library is designed to provide core mobility functions for
an autonomous two-wheeled robotic vehicle using stepper motors 
controlled using the MD_Stepper library.

This library provides the code infrastructure that allows the 
car to travel in a controlled manner, on top of which specific 
applications can confidently be built.

This library is designed around a custom designed two wheel drive 
(+ idler castor wheel) vehicle chassis depicted below. The software 
also be suitable, with little or no modifications, for more capable 
platforms with similar mechanisms.

![SmartCar2 Platform] (SmartCar2_Platform.png "SmartCar2 Platform")

The vehicle hardware and control system are made up of a number of
subcomponents that are functionally brought together by the software
library:
- \subpage pageVehicleHardware
- \subpage pageMotorController

The control hierarchy implemented in the library is shown in the figure
below. The library implements the control elements from "Motion Control" 
to the right of the figure. The components to the left of 'Motion Control' 
are defined into the application that defines the vehicle's behavior.

![Control Hierarchy] (SmartCar2_Control_Hierarchy.png "Control Hierarchy")

The library is designed to control 2 types of autonomous movements:
- _Precisely controlled movements_ (eg, spin in place), where the ability to
  manoeuvre the orientation of the vehicle at low speed is important. 
  Independent control of motor directions and how far it spins are used as 
  control parameters for this mode type of movement.
- _General movements_ (eg, traveling at a set speed in a set direction),
  where the ability to move more quickly in an specified path is important. 
  This type of movement is managed using the \ref pageControlModel "unicycle 
  model" for control coupled to \ref pagePID "PID control" of the DC motors. 

### Library Topics
- \subpage pageVehicleHardware
- \subpage pageHardwareMap
- \subpage pageMotorController
- \subpage pageUsingLibrary
- \subpage pageControlModel
- \subpage pageActionSequence

### Additional Topics
- \subpage pageRevisionHistory
- \subpage pageDonation
- \subpage pageCopyright

### Library dependencies
- MD_cmdProcessor library located at https://github.com/MajicDesigns/MD_cmdProcessor or the Arduino library manager
- MD_Stepper library is located at https://github.com/MajicDesigns/MD_Stepper or the Arduino library manager

\page pageDonation Support the Library
If you like and use this library please consider making a small donation 
using [PayPal](https://paypal.me/MajicDesigns/4USD)

\page pageCopyright Copyright
Copyright (C) 2021 Marco Colli. All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

\page pageRevisionHistory Revision History
Nov 2021 version 1.0.0
- Initial release created from MD_SmartCar library code
 */

#include <Arduino.h>
#include <MD_Stepper.h>

 /**
 * \file
 * \brief Main header file and class definition for the MD_SmartCar2 library.
 */

#ifndef SCDEBUG
#define SCDEBUG  0    ///< set to 1 for library debug output
#endif

#if SCDEBUG
#define SCPRINT(s,v)   do { Serial.print(F(s)); Serial.print(v); } while (false)
#define SCPRINTX(s,v)  do { Serial.print(F(s)); Serial.print(F("0x")); Serial.print(v, HEX); } while (false)
#define SCPRINTS(s)    do { Serial.print(F(s)); } while (false)
#else
#define SCPRINT(s,v)
#define SCPRINTX(s,v)
#define SCPRINTS(s)
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#endif

/**
 * Core object for the MD_SmartCar2 library
 */
class MD_SmartCar2
{
public:
  //--------------------------------------------------------------
  /** \name Structures, Enumerated Types and Constants.
   * @{
   */
  /**
    * Maximum number of motors
    *
    * Define the maximum number of motors that this library will control
    */
  static const uint8_t MAX_MOTOR = 2;

  /**
   * Enumerated type for Action Items operation
   * 
   * Specifies which operation is being defined in the actionItem_t
   */
  enum actionId_t
  {
    DRIVE,    ///< executes drive(); param 0 lin vel, param 1 angular velocity
    MOVE,     ///< executes a move(); param 0 left rotate, param 1 right rotate
    SPIN,     ///< executes a spin(); param 0 spin percentage
    PAUSE,    ///< executes a pause; param 0 milliseconds pause
    STOP,     ///< executes a stop()
    END       ///< marks the end of the action list; should always be last item.
  };
  
  /**
    * Move sequence item definition
    * 
    * Define one of the action elements for a move() sequence.
    */
  typedef struct 
  {
    actionId_t opId;          ///< id for the action specified by this item
    float parm[MAX_MOTOR];    ///< function parameter
  } actionItem_t;

  /** @} */

  //--------------------------------------------------------------
  /** \name Class constructor and destructor.
   * @{
   */
  /**
   * Class Constructor
   *
   * Instantiate a new instance of the class.
   * This variant is for motor controllers that have a PWM input for speed control.
   *
   * The main function for the core object is to reset the internal
   * shared variables and timers to default values.
   *
   * \param ml The MD_Stepper object for controlling the left side motor.
   * \param mr The MD_Stepper object for controlling the right side motor.
   */
  MD_SmartCar2(MD_Stepper* ml, MD_Stepper* mr);

  /**
   * Class Destructor.
   *
   * Release any allocated memory and clean up anything else.
   */
  ~MD_SmartCar2(void);
  /** @} */

  //--------------------------------------------------------------
  /** \name Methods for core object control.
   * @{
   */
   /**
    * Initialize the object.
    *
    * Initialize the object data. This needs to be called during setup() to reset new
    * items that cannot be done during object creation.
    *
    * Vehicle constants are passed through to the setVehicleParameters() method. See
    * comments for that method for more details.
    *
    * \sa setVehicleParameters();
    *
    * \param ppr    Number of encoder pulses per wheel revolution.
    * \param ppsMax Maximum number of encoder pulses per second at top speed (100% velocity).
    * \param dWheel Wheel diameter in mm.
    * \param lBase  Base length (distance between wheel centers) in mm
    * \return false if either encoder did not reset, true otherwise.
    */
  bool begin(uint16_t ppr, uint16_t ppsMax, uint16_t dWheel, uint16_t lBase);

  /**
   * Set the vehicle constants
   *
   * Sets the number of pulses per encoder revolution, maximum speed reading and
   * important dimensions for the vehicle. This depends on the hardware and could
   * vary between different vehicle configurations.
   *
   * For encoder ppr, there is only one value for all whole vehicle, so all
   * encoders need to operate the same way.
   * 
   * \sa begin(), \ref pageUsingLibrary
   *
   * \param ppr    Number of encoder pulses per wheel revolution.
   * \param ppsMax Maximum number of encoder pulses per second at top speed (100% velocity).
   * \param dWheel Wheel diameter in mm.
   * \param lBase  Base length (distance between wheel centers) in mm
   */
  void setVehicleParameters(uint16_t ppr, uint16_t ppsMax, uint16_t dWheel, uint16_t lBase);

  /**
   * Run the Robot Management Services.
   *
   * This is called every iteration through loop() to run all the required
   * Smart Car Management functions.
   */
  void run(void);

  /**
   * Check if motors are running
   *
   * Check if motors are commanded to run. This method is useful to check when
   * drive() or move() have completed their motions.
   *
   * \return true if any of the motors are not idle
   */
  bool isRunning(void);

  /**
   * Check if specific motor is running
   *
   * Check if motors are commanded to run. This method is useful to check when
   * drive() or move() have completed their motions.
   *
   * \param mtr  The motor number being queried [0..MAX_MOTOR-1]
   * \return true if any of the motors are not idle
   */
  bool isRunning(uint8_t mtr) { return(mtr < MAX_MOTOR ? _mData[mtr].motor->isBusy() : false); }
  /** @} */

  //--------------------------------------------------------------
  /** \name Methods for free running the vehicle.
   * @{
   */
   /**
    * Drive the vehicle along specified path (degrees).
    *
    * Run the vehicle along a path with the specified velocity and angular orientation.
    *
    * The velocity is specified as a percentage of the maximum vehicle velocity [0..100].
    * Positive velocity move the vehicle forward, negative moves it in reverse. The speed
    * will be adjusted using the acceleration parameters.
    *
    * Angular velocity is specified in degrees per second [-90..90]. Positive angle
    * is clockwise rotation.
    *
    * \sa getLinearVelocity(), getAngularVelocity(), setAccelProfile()
    *
    * \param vLinear   the linear velocity as a percentage of full scale [-100..100].
    * \param vAngularD the angular velocity in degrees per second [-90..90].
    */
  void drive(int8_t vLinear, int8_t vAngularD) { drive(vLinear, deg2rad(vAngularD)); }

  /**
   * Drive the vehicle along a straight path.
   *
   * Run the vehicle along a straight path with the specified velocity.
   *
   * The velocity is specified as a percentage of the maximum vehicle velocity [-100..100].
   * Positive velocity move the vehicle forward, negative moves it in reverse. The speed
   * will be adjusted using the acceleration parameters.
   *
   * \sa getLinearVelocity(), setAccelProfile()
   *
   * \param vLinear   the linear velocity as a percentage of full scale [-100..100].
   */
  void drive(int8_t vLinear) { drive(vLinear, (float)0); }

  /**
   * Drive the vehicle along specified path (radians).
   *
   * Run the vehicle along a path with the specified velocity and angular orientation.
   *
   * The velocity is specified as a percentage of the maximum vehicle velocity [-100..100].
   * Positive velocity move the vehicle forward, negative moves it in reverse. The speed
   * will be adjusted using the acceleration parameters.
   *
   * Angular velocity direction is specified in radians per second [-pi/2..pi/2]. Positive
   * angle is clockwise rotation.
   *
   * \sa getLinearVelocity(), getAngularVelocity(), setAccelProfile()
   *
   * \param vLinear   the linear velocity as a percentage of full scale [-100..100].
   * \param vAngularR the angular velocity in radians per second [-pi/2..pi/2].
   */
  void drive(int8_t vLinear, float vAngularR);

  /**
   * Stop the smart car.
   *
   * This method brings the SmartCar to an immediate stop.
   *
   * \param insideSequence set true if this was called from inside a sequence (not for end users)
   * 
   * \sa setSpeed()
   */
  void stop(bool insideSequence = false);

  /**
   * Set the linear velocity
   *
   * Sets the linear velocity without changing any other parameters. Useful for
   * adjusting the speed when already in motion. The vehicle will accelerate/decelerate
   * to the new speed.
   *
   * The velocity is specified as a percentage of the maximum vehicle velocity [-100..100].
   * Positive velocity move the vehicle forward, negative moves it in reverse.
   *
   * /sa getLinearVelocity(), drive(), setAccelProfile()
   *
   * \param vel the new value for the linear velocity [-100..100].
   */
  void setLinearVelocity(int8_t vel) { drive(vel, _vAngular); } 

  /**
   * Get the current linear velocity.
   *
   * Linear velocity is expressed as a percentage of the maximum velocity [-100..100].
   * The Master velocity is used to regulate all the speed functions for the motors.
   *
   * \sa drive()
   *
   * \return the current linear speed setting.
   */
  inline int8_t getLinearVelocity(void) { return(_vLinear); }

  /**
   * Set the angular velocity (radians).
   *
   * Sets the angular velocity without changing any other parameters. Useful for
   * adjusting turning when already in motion.
   *
   * Angular velocity is expressed in radians relative to the forward direction
   * [-PI/2..PI/2]. Positive angle is turn to the right, negative left.
   *
   * \sa getAngularVelocity(), drive()
   *
   * \param angR the new turning rate in radians.
   */
  inline void setAngularVelocity(float angR) { drive(_vLinear, angR); }

  /**
   * Set the angular velocity (degrees).
   *
   * Sets the angular velocity without changing any other parameters. Useful for
   * adjusting turning when already in motion.
   *
   * Angular velocity is expressed in degrees relative to the forward direction
   * [-90..90]. Positive angle is turn to the right, negative left.
   *
   * \sa getAngularVelocity(), drive()
   *
   * \param angD the new turning rate in degrees.
   */
  inline void setAngularVelocity(int8_t angD) { drive(_vLinear, deg2rad(angD)); }

  /**
   * Get the current angular velocity.
   *
   * Angular velocity is expressed in radians relative to the forward direction
   * [-PI/2..PI/2]. Positive angle is turn to the right, negative left.
   *
   * \sa drive()
   *
   * \return the current angular speed setting.
   */
  inline float getAngularVelocity(void) { return(_vAngular); }

  /**
  * Set the acceleration profile parameters
  * 
  * To make the best use of the stepper motor torque, speed changes are adjusted in 
  * incremental steps. The number of stepos is given by the __steps__ parameter and 
  * the time between each step is given by the __timebase__ parameter.
  *
  * The library calculates the difference between the 2 speed (S2 - S1) and divides 
  * this by __steps__. During the acceleration/deceleration, the motor speed is 
  * changed by this quantity __timebase__ time period until the new speed is reached.
  * 
  * Default values are the class private constants DEF_ACCEL_TIME and DEF_ACCEL_STEPS.
  *
  * \param timebase time between acceleration steps (in milliseconds)
  * \param steps    number of dicrete steps between start and end speeds
  */
  void setAccelProfile(uint16_t timebase, uint8_t steps);

  /** @} */

  //--------------------------------------------------------------
  /** \name Methods for precision movements of the vehicle.
   * @{
   */
   /**
   * Precisely move the vehicle (radians).
   *
   * Controls the movement by counting the encoder pulses rather that PID,
   * which should make it more precise and controlled. This is useful for specific
   * movements run at slow speed.
   *
   * The call to move() specifies the angle each wheels will turn, independently.
   * This method is designed to allow close movements such as spin-in-place or
   * other short precise motions.
   *
   * The motion for each wheel is specified as speed as the total angle subtended
   * by the turned by the wheel in radians. Negative angle is a reverse wheel
   * rotation.
   *
   * \sa drive(), spin(), len2rad()
   *
   * \param angL left wheel angle subtended by the motion in radians.
   * \param angR right wheel angle subtended by the motion in radians.
   */
  void move(float angL, float angR);

  /**
  * Precisely move the vehicle (degrees).
  *
  * Controls the movement by counting the encoder pulses rather that PID,
  * which should make it more precise and controlled. This is useful for specific
  * movements run at slow speed.
  *
  * The call to move() specifies the precise motion of the motors, independently.
  * This method is designed to allow close movements such as spin-in-place or
  * other short precise motions.
  *
  * The motion for each wheel is specified as speed as the total angle subtended
  * by the turned by the wheel in degrees. Negative angle is a reverse wheel rotation.
  *
  * \sa drive(), spin()
  *
  * \param angL left wheel angle subtended by the motion in degrees.
  * \param angR right wheel angle subtended by the motion in degrees.
  */
  void move(int16_t angL, int16_t angR) { move(deg2rad(angL), deg2rad(angR)); }

  /**
  * Precisely move the vehicle (millimeter).
  *
  * Controls the movement by counting the encoder pulses rather that PID,
  * which should make it more precise and controlled. This is useful for specific
  * movements run at slow speed.
  *
  * The call to move() specifies the precise motion of the motors, independently.
  * This method is designed to allow close movements such as spin-in-place or
  * other short precise motions.
  *
  * The motion for each wheel will be identical to move the vehicle the required
  * distance. Negative length is a reverse wheel rotation.
  *
  * \sa drive(), spin()
  *
  * \param len distance to move in mm.
  */
  void move(int16_t len) { move(len2rad(len), len2rad(len)); }

  /**
  * Precisely spin the vehicle.
  *
  * Controls the movement by spinning the vehicle about its central vertical
  * axis. It works similar to move() to spin the wheels in an directions to
  * effect the turning motion.
  *
  * The call to spin() specifies the percentage (-100 to 100) of the full circle
  * rotation about the central axis passing through the vehicle base length.
  * Positive angle is a turn to the right, negative to the left.
  *
  * \sa drive(), move()
  *
  * \param fraction Percentage fraction of full revolution [-100..100]. Positive spins right; negative pins left.
  */
  void spin(int16_t fraction);

  /**
   * Start an action sequence stored in PROGMEM.
   * 
   * This method is passed the reference to an action sequence array stored in PROGMEM
   * to the library for background execution.
   * 
   * Details on actions sequences can be found at \ref pageActionSequence
   * 
   * \sa isSequenceComplete()
   * 
   * \param actionList pointer to the array of actionItem_t ending with and END record.
   */
  void startSequence(const actionItem_t* actionList);

  /** 
   * Start an action sequence stored in RAM.
   * 
   * This method is passed the reference to an action sequence array stored in RAM
   * to the library for background execution. The array must remain in scope
   * (ie, global or static declaration) for the duration of the sequence 
   * running.
   *
   * Details on actions sequences can be found at \ref pageActionSequence
   *
   * \sa isSequenceComplete()
   *
   * \param actionList pointer to the array of actionItem_t ending with and END record.
   */
  void startSequence(actionItem_t* actionList);

  /**
   * Stop the currently executing sequence.
   *
   * This method stops the currently executing sequence in the middle of
   * the execution stream. This vehicle will be left in an indetermined state
   * and the sequence will be reported as completed.
   *
   * \sa isSequenceComplete()
   */
  void stopSequence(void);

  /**
   * Check if the current action sequence has completed.
   * 
   * Once an action sequqnce is started it will automatically execute to 
   * completion unless interrupted. This method checks to see if the
   * action sequqnce has completed.
   * 
   * \sa startSequence()
   * 
   * \return true if the sequence has finished executing
   */
  bool isSequenceComplete(void) { return(!_inSequence); }

  /** @} */
  //--------------------------------------------------------------
  /** \name Methods for Configuration Management.
   * @{
   */
  /**
   * Read pulses per wheel revolution
   *
   * Returns the number of pulses per encoder revolution. This may be needed to 
   * change from number of pulses to revolutions and then distance.
   *
   * \sa setVehicleParameters()
   *
   * \return The number of pulses per revolution.
   */
  inline uint16_t getPulsePerRev() { return(_ppr); }

  /**
   * Read linear length moved per wheel pulse
   *
   * Returns the distance traveled for each pulse of the motor.
   *
   * \sa setVehicleParameters()
   *
   * \return The distance travelled with each pulse.
   */
  inline float getDistancePerPulse() { return(_lenPerPulse); }

  /** @} */
  //--------------------------------------------------------------
  /** \name Utility methods.
   * @{
   */
  /**
   * Convert degrees to radians.
   *
   * Convert the degrees measure specified into radians.
   *
   * \param deg the value in degrees to be converted.
   * \return the converted value
   */
  inline float deg2rad(int16_t deg) { return((PI * (float)deg) / 180.0); }

  /**
   * Convert a length to angle of wheel rotation.
   * 
   * Convert a length in mm to travel into the radian of wheel rotation 
   * required for that travel.
   * 
   * \param len length in mm to convert.
   * \return the angle in radians of wheel rotation to achieve that distance.
   */
  inline float len2rad(int16_t len) { return(((float)len * 2 * PI) / (_lenPerPulse * _ppr)); }

  /** @} */

private:
  const uint32_t DEF_ACCEL_TIME = 10;  // time between acceleration steps (ms)
  const uint8_t DEF_ACCEL_STEPS = 16;  // number of steps between start and end speeds
  const uint8_t MOVE_SPEED = 40;       // default move speed for vehicle (% full speed)

   // Motor array indices
  const uint8_t MLEFT = 0;      ///< Array index for the Left motor
  const uint8_t MRIGHT = 1;     ///< Array index for the right motor

  enum runState_t 
  { 
    S_IDLE,         ///< Motor is idle
    S_ACCEL_INIT,   ///< Motor initializing acceleration from current to setpoint speed
    S_ACCEL,        ///< Motor acceleration from speed to setpoint speed
    S_DRIVE_RUN,    ///< Motor running free at setpoint speed
    S_DECEL_INIT,   ///< Motor initializing decelerating from speed to setpoint speed
    S_DECEL,        ///< Motor decelerating from speed to setpoint speed

    S_MOVE_INIT,    ///< Motor initializing to move run from idle
    S_MOVE_RUN      ///< Motor running in move mode (step counts)
  };

  float _vMaxLinear;      ///< Maximum linear speed in pulses/second 
  int16_t _vLinear;       ///< Master velocity as percentage [-100..100] = [-_vMaxLinear.._vMaxLinear]
  float _vAngular;        ///< Angular velocity setpoint in in radians per second [-PI..PI]

  uint16_t _accelTime;    ///< acceleration profile time base
  uint8_t _accelSteps;    ///< number of steps between speeds

  // Vehicle constants
  uint16_t _ppr;          ///< Encoder pulses per wheel revolution
  uint16_t _ppsMax;       ///< Maximum speed in pulses per second
  uint16_t _diaWheel;     ///< Wheel diameter in mm
  uint16_t _lenBase;      ///< Base length in mm (distance between wheel centers)

  float _lenPerPulse;     ///< Length traveled per pulse of wheel revolution
  float _diaWheelP;       ///< Wheel diameter in pulses (calculated)
  float _lenBaseP;        ///< Base Length in pulses (calculated)

  // Data for tracking action sequences
  bool _inSequence;       ///< true if currently executing a sequence
  bool _inAction;         ///< waiting for current item to complete
  bool _seqIsConstant;    ///< true if sequence is stored is declared in PROGMEM
  union 
  {                 
    const actionItem_t* cp; 
    actionItem_t* p;
  } _uAction;             ///< current list of actions being sequenced
  uint8_t _curActionItem; ///< index for the current action item
  actionItem_t _ai;       ///< current action item
  uint32_t _timeStartSeq; ///< generic time variable for sequences

  // Motor state data used to manage each motor
  struct motorData_t
  {
    MD_Stepper  *motor;    ///< motor controller

    uint32_t    timeMark;  ///< generic time marker used as required
    int32_t     speedSP;   ///< setpoint speed in pulses/s
    int32_t     speedCV;   ///< current speed in pulse/s
    int32_t     pulse;     ///< move pulses set (+/-) or pulse inc/decrement for acceleration
    runState_t  state;     ///< control state for this motor
  };
  
  motorData_t _mData[MAX_MOTOR];  ///< keeping track of each motor's parameters

  // Private Methods
  void startSeqCommon(void);            ///< common part of sequence start
  void runSequence(void);               ///< keep running current sequence
  bool runActionItem(actionItem_t& ai); ///< run the logic for this action item
};
