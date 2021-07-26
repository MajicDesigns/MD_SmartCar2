#include <MD_SmartCar2.h>

/**
 * \file
 * \brief Code file for SmartCar2 library class.
 */

/**
\page pageUsingLibrary Setting up a new SmartCar2

Starting with new SmartCar2 chassis and computer hardware, it is a relatively
straightforward process to work out the right configuration parameters to
get the vehicle set up:

-# \ref pageSetupConstants
-# \ref pageSetupMotor
-# \ref pageSetupConfig
-# \ref pageSetupPID
-# \ref pageSetupControl

\page pageSetupConstants Measure Physical Constants

An application using the library needs to pass a few physical constants to the 
MD_SmartCar2::begin() method that allows the library to configure vehicle control
parameters.

Note that the units of length are specified in millimeters. However, alternative 
units (eg inches or centimeters) may be used __AS LONG AS ALL LENGTHS ARE 
SPECIFIED IN THE SAME UNITS__, as the units all cancel out in the calculations.

Two physical constants need to be directly measured from the vehicle:
- The __wheel diameter__ in mm. This is measured to the outer edge of the wheel tyre.
- The __base length__ (distance between wheel centers) in mm. Inside edge of one wheel 
to the inside edge of the other wheel.

These two constants are shown as _2r_ and _B_ in the figure below.

![SmartCar2 Unicycle](SmartCar2_Unicycle.png "SmartCar2 Distances Measured")

- The number of __encoder pulses per wheel revolution__. This is measured by counting
the gaps, or white spaces, of the encoder wheel and doubling the count, as one pulse 
is received for each edge transition of the encoder.

![SmartCar2 Encoder](SmartCar2_Encoder_Wheel.jpg "SmartCar2 Encoder Wheel")

- The __maximum number of encoder pulses per second__ at top speed (100% velocity). 
This can be measured later, with the motors running, in the steps verifying the 
correctness of motor wiring and setup.

Next: \ref pageSetupMotor
____

\page pageSetupMotor Confirming Motor Setup

The first part of setup is to use the __MotorTest__ example sketch to ensure 
that the motors are turning in the correct direction and the encoders are 
working correctly.

MotorTest allows control of the vehicle motors using the Serial Monitor through 
the command line interface to invoke test the functions (type ? for help text 
listing the functions).

Independently commanding the motors to move in a Forward and Reverse direction 
provides confirmation that they are wired correctly. If they rotate the wrong 
(opposite) direction, simply reverse the wiring between the speed controller 
and motor or between the processor I/O pins and the speed controller.

The encoder feedback can also be tested now - you are looking for the software 
to print counts for both encoders when the motors are running. 

The encoder feedback is printed every second, so the maximum number of pulses per 
second is worked out by observing the counts and recording the lowest of the two 
motor counts reported at the maximum speed setting (PWM 255). It is normal for 
the counts to be different due to differences in the electrical and mechanical 
characteristics of the two motors.

At this stage it is also worth experimenting and noting 2 additional PWM settings:
- The lowest viable PWM setting that will keep a motor turning when it has already
started. This is worked out by starting from a high value and slowly reducing until 
the motor no longer turns. This value is a good starting point to set the low value 
for PID control limits in the next setup stage.
- The PWM value needed to start a motor. Internal mechanical and electrical friction 
means that there is a minimum threshold below which the PWM is insufficient to get a 
motor started. This value is a lower bound for the 'kicker' setting in the next 
configuration step.

Once again, these values may be different for each motor and in this case you need to 
pick the highest of the two PWM values.

Next: \ref pageSetupConfig
____

\page pageSetupConfig Determining Calibration Parameters

The next step is to use the __Calibrate__ example sketch to determine configuration 
parameters and store them permanently in EEPROM.

__Calibrate__ allows the vehicle motors to be controlled through the Serial Monitor 
using a command line interface (CLI) to invoke test the functions (type ? for 
help text listing the functions).

Whilst setting parameters, it is a good idea to save parameters to EEPROM using 
the CLI on a regular basis. The saved parameters can be reloaded from EEPROM at 
any time - useful when a change has created an unviable set of parameters.

#### Setting MD_SmartCar2::drive() kicker PWM
The drive() method initiates PID speed control of the motors. This may include 
speeds that are less than the minimum needed to rotate the motor from a standing
start. 

For low speeds and a standing start, the kicker PWM is set as the initial speed 
for a short time to 'unstick' the motor before the actual (lower) speed is set. 
The kicker PWM should be set slightly higher than the PWM value found when setting 
up the motors.

#### Setting MD_SmartCar2::move() PWM
The move() function allows precise movements at this predefined PWM setting. The 
setting is a compromise that needs to be fine tuned by experimentation, and should
- be higher than the kicker setting to enable the motor to start with no problems
- be lower than a value that creates too much free movement (due to vehicle inertia)
at the end of the move().

#### Setting PWM control limits

The control limits are the lowest and highest outputs allowed by the PID controller.
The lower limit should have been noted in the first step, using __MotorTest__. The 
lower limit should also be less than either the kicker or move() PWM settings.

The upper limit will generally be set at 255 (the maximum) unless there is some other 
reason to make it lower.

Next: \ref pageSetupPID
____

\page pageSetupPID Tuning PID Parameters

The next step is to use the __Calibrate__ example sketch and the SerialStudio 
application (found at https://www.alex-spataru.com/serial-studio.html) to determine 
the motor control PID parameters. The IDE Serial Monitor can be used to view the 
raw numbers output from the library whilst tuning the PID loop; SerialStudio provides 
superior data visualization and an easier tuning process.

__Calibrate__ allows the vehicle motors to be controlled through the Serial Monitor 
using a command line interface (CLI) to invoke test the functions (type ? for help 
text listing the functions). SerialStudio replaces the IDE Serial Monitor and 
adds real-time graphical plots of the PID control parameters.

Whilst setting parameters, it is a good idea to save to EEPROM using the CLI when a 
good set of Kp, Ki and Kd parameters are found. The saved parameters can be reloaded 
from EEPROM at any time, especially when changes make the PID tuning worse.

#### Setting up for PID Tuning
-# Edit MD_SmartCar2.h and change 
   \code #define PID_TUNE 0 \endcode to 
   \code #define PID_TUNE 1 \endcode 
   This will enable the library to output PID control parameters as JSON data. Once 
tuning is completed, this defined value should be changed back to 0 to suppress output.
-# Compile and download the __Calibrate__ sketch with this setting turned on.
-# Start the SerialStudio application and configure it as shown in the figure below
  - Set the Serial Parameters - COM port and baud rate - in the red highlight box. 
  - Set the JSON file to 'Manual' and choose the _PID.json_ file in the library _src_
  folder (orange highlight box). Check the Settings tab and set the Start and End 
  sequences to '{' and '}'. This will make SerialStudio recognise the PID output data 
  packets in the received Serial stream.
  - Connect the serial port to the vehicle (yellow highlight box).

![SmartCar2 SerialStudio Setup](SmartCar2_SerialStudio.png "SmartCar2 SerialStudio setup")

SerialStudio has the same input functionality as the Serial Monitor. You can type 
commands to the vehicle application using the console input box (light blue highlight). 
The figure shows the result of the '?' command (help text).

#### PID Tuning
Once SerialStudio is set up, starting the motors from the console will automatically 
switch to a graphical dashboard view, shown below. This allows you to visualize the 
effects of changes to the speed setpoints and/or the PID parameters. You can switch 
between console and dashboard views using the menu options at the top of the window.

Parameters shown are Set Point (SP), Current Value (CV) and control Output (CO) for the
Left and Right motors.

![SmartCar2 SerialStudio Dashboard](SmartCar2_PIDsetup.png "SmartCar2 SerialStudio Dashboard")

The default PID values in the SmartCar2_HW header file should be a good starting point 
for the PID motor settings - expect to have a high Kp, no Ki and a relatively small Kd 
(this is really a PD controller). PID values can be set independently for Left and Right 
motors if required. In most cases the same PID setup for both motors should be adequate.

Once you are happy with the performance of the PID control loop, save the parameters to 
EEPROM.

Next: \ref pageSetupControl
____

\page pageSetupControl Testing with Remote Control

This final phase is all about testing the vehicle 'in real life' as it moves about on the floor. 
The dynamics of this situation are likely to be different from the bench testing previously 
done, so some additional tuning may be necessary.

The step uses the __Setup_Control__ example sketch and the related App Inventor 2 (AI2, see 
http://ai2.appinventor.mit.edu/) "Setup Control" application found in the example sketch folder.
The AI2 application provides a GUI front end for commands through a Bluetooth interface. The 
same commands could be issued from the Serial Monitor (or other Terminal program) connected 
through a Bluetooth serial port.

The AI2 application has a main menu leading to a displays for controlling drive() and move(),
a Terminal to monitor messages from the vehicle and a setup display for changing config 
parameters. The current parameter settings are shown in the setup screen's terminal window 
and can be changed from there.

![SmartCar2 AI2 Setup Control](SmartCar2_AI2SetupControl.jpg "SmartCar2 AI2 Setup Control")

#### Tuning spin() derating factor
This is the last of the setup parameters. When the vehicle executes a spin() and then stops,
it will continue to move for a short time due to angular inertia. 
This parameter is a 'fudge' factor to stop the motion before the actual end (derate) so that 
the vehicle will coast to approximately the right spot. The factor is the fraction of the full 
rotation motion (for example, 0.7 will stop the motion 70% through the required steps).

This parameter set up is a compromise between long rotations (more momentum) and short ones 
(less momentum). 

#### Checking MD_SmartCar2::drive() kicker, MD_SmartCar2::move() and PID parameters
A final check of these parameters in action with the vehicle moving its own weight around. The 
parameters can be modified from the setup screen of ther AI2 app if they need further tuning.

\page pageControlModel Unicycle Control Model

Working out the displacement and velocities of each wheel on a
differential drive robot can be messy.

The _unicycle model_ of an autonomous robot is a kinematic
model that allows modeling the movement of the vehicle
as if it was a unicycle, using a linear velocity vector (V)
and a rotational velocity (&omega;) about a point within the
vehicle. Taking this point to be midway between axis joining 
the 2 wheels simplifies the calculation.

![SmartCar2 Movement Transform](SmartCar2_Transform.png "SmartCar2 Movement Transform")

We can derive equations that translate between the unicycle model
and our wheel velocities. Steering requires each of the independent
wheels to rotated at different speeds (V<sub>L</sub> and V<sub>R</sub> for the
left and right side) to travel the equivalent unicycle path.

So specifying a movement path using this abstraction becomes much easier
as we need to just specify "How fast do we want to move forward and
how fast do we want to turn", letting the mathematics work out the
wheel rotations.

![SmartCar2 Unicycle](SmartCar2_Unicycle.png "SmartCar2 Unicycle Model")

V and &omega; are transformed into independent motor speeds for the left
and right motor (V<sub>L</sub>, V<sub>R</sub>) using the following formulas:
- V<sub>L</sub> = (2V + &omega;B) / 2r
- V<sub>R</sub> = (2V - &omega;B) / 2r

where B is the vehicle base length (ie, the distance between the wheel 
centerlines) and r is the radius of the wheel.

The convention used in this library is:
- Linear velocity V is positive for forward motion, negative backwards.
- Angular velocity &omega; is positive for right rotation, negative for left.

![SmartCar2 Convention](SmartCar2_Convention.png "SmartCar2 Library Convention")
____
### For more details
- http:://faculty.salina.k-state.edu/tim/robotics_sg/Control/kinematics/unicycle.html
- https://www.youtube.com/watch?v=aSwCMK96NOw&list=PLp8ijpvp8iCvFDYdcXqqYU5Ibl_aOqwjr

\page pageActionSequence Action Sequences

Action sequences are a way of defining a sequential actions that the library 
will execute to move the vehicle (ie, a 'recipe' for movement). They save time
in not having to program common combinations of actions and monitoring each 
completion in the application sketch. The library executes the action sequence 
in the background freeing the application to run other higher priority tasks.

An simple example is when a front bump switch detects a collision, triggering 
the evasive action defined by:
- stop the vehicle
- pause a short while
- reverse away from the obstacle
- pause a short while
- spin to another direction
- resume normal vehicle motion

These actions can be defined in a list of actions to be performed and passed to
the library for execution:
\code
static const PROGMEM MD_SmartCar2::actionItem_t seq[] =
{
  { MD_SmartCar2::STOP },
  { MD_SmartCar2::PAUSE, 300 },
  { MD_SmartCar2::MOVE,  -PI, -PI },
  { MD_SmartCar2::PAUSE, 300 },
  { MD_SmartCar2::SPIN,  -25 },
  { MD_SmartCar2::END }
};
\endcode

The action sequence is defined as an array of MD_SmartCar2::actionItem_t
records. Each record contains the action to be performed and the parameters 
relevant to that action (summarized in the table below). The last record
in the array must always be the END action or the library will continue
reading random memory beyond the end of the sequence.

Sequences may be completely predefined, allowing them to be stored in 
static (PROGMEM) memory to preserve dynamic RAM, or they may be built 
and/or modified 'on the fly' in RAM.

The list of actions that can be defined an actionItem_t are listed given by the
enumerated type MD_SmartCar2::actionId_t.

|         ActionId_t | Comment          | Parameter 0     | Parameter 1
|-------------------:|:-----------------|:----------------|:------------
| MD_SmartCar2::DRIVE | executes drive() | Linear Velocity | Angular Velocity
|  MD_SmartCar2::MOVE | executes move()  | Left rotate     | Right rotate
|  MD_SmartCar2::SPIN | executes spin()  | Spin percentage | Not used
| MD_SmartCar2::PAUSE | executes pause   | Milliseconds    | Not used
|  MD_SmartCar2::STOP | executes stop()  | Not used        | Not used
|   MD_SmartCar2::END | marks seq end    | Not used        | Not used

*/

MD_SmartCar2::MD_SmartCar2(AccelStepper *ml, AccelStepper *mr)
{
  // Allocate the pointer to the right array reference
  _mData[MLEFT].motor = ml;
  _mData[MRIGHT].motor = mr;
}

MD_SmartCar2::~MD_SmartCar2(void) 
{
}

bool MD_SmartCar2::begin(uint16_t ppr, uint16_t maxSpeed, uint16_t dWheel, uint16_t lBase)
// Return false if anything fails
{ 
  bool b = true;

  _inSequence = false;

  // Set up default environment
  setVehicleParameters(ppr, maxSpeed, dWheel, lBase);
  stop();    // initialize to all stop

  return(b);
}

void MD_SmartCar2::run(void)
// run the FSM to manage motor functions
{
  bool firstPass = true;

  // run the sequence to set up a command if we are currently in that mode
  if (_inSequence)
    runSequence();

  // loop through all the motors doing whatever in each state
  for (uint8_t motor = 0; motor < MAX_MOTOR; motor++)
  {
    switch (_mData[motor].state)
    {
    case S_IDLE: break;     // do nothing

    // --- FREE RUNNING
    case S_DRIVE_INIT:    // initialise driving sequence
      SCPRINT("\n>>DRIVE_INIT #", motor);
      _mData[motor].motor->setCurrentPosition(0);
      _mData[motor].motor->setSpeed(_mData[motor].speed);
      _mData[motor].state = S_DRIVE_RUN;
      // deliberately fall through

    case S_DRIVE_RUN:     // run at constant speed
      _mData[motor].motor->runSpeed();
      if (_mData[motor].motor->speed() == 0)
      {
        SCPRINT("\n>>DRIVE_RUN to IDLE #", motor);
        _mData[motor].state = S_IDLE;
      }
      break;

    // --- PRECISION MOVES
    case S_MOVE_INIT:
      SCPRINT("\n>>MOVE_INIT #", motor);
      _mData[motor].motor->setCurrentPosition(0);
      _mData[motor].motor->setSpeed(_mData[motor].speed);
      _mData[motor].motor->moveTo(_mData[motor].pulse);
      _mData[motor].state = S_MOVE_RUN;
      // deliberately fall through

    case S_MOVE_RUN:
      if (firstPass)
      {
        firstPass = false;
        SCPRINTS("\nMOVE");
      }
      else
        SCPRINTS(",");
      SCPRINT(" [", motor); 
      SCPRINT("] ", _mData[motor].motor->distanceToGo());
      SCPRINT("/", _mData[motor].pulse);
        
      // run the steps and check for ending conditions
      _mData[motor].motor->run();
      if (_mData[motor].motor->distanceToGo() == 0 || _mData[motor].motor->speed() == 0)
      {
        SCPRINT("\n>>MOVE_RUN to IDLE #", motor);
        _mData[motor].state = S_IDLE;
      }
      break;

    default: _mData[motor].state = S_IDLE; break;
    }
  }
}

void MD_SmartCar2::drive(int8_t vLinear, float vAngularR)
{
  float spL, spR;

  if (vLinear == 0)
    stop();
  else if ((vLinear == _vLinear) && (vAngularR == _vAngular))
    return;    // no change
  else
  {
    SCPRINT("\n** DRIVE v:", vLinear);
    SCPRINT(" a:", vAngularR);

    // sanitize input
    if (vLinear < -100) vLinear = -100;
    if (vLinear > 100) vLinear = 100;
    if (vAngularR < -PI/2) vAngularR = -PI/2;
    if (vAngularR > PI/2)  vAngularR = PI/2;

    // save these for reporting/other use
    _vLinear = vLinear;
    _vAngular = vAngularR;
    
    // set up for calculations
    vAngularR = -vAngularR;  // reverse library the convention for calcs

    // Unicycle control kinematics differential wheel velocity
    // vL = (2v - wL)/(D); vR = (2v + wL)/(D)
    // where 
    // vL, vR are left and right velocity of wheel in encoder pulse/sec
    // v = linear velocity of vehicle (vLinear)
    // w = angular velocity of vehicle (vAngular)
    // L = vehicle wheel Base (_lenBase converted to _lenBaseP)
    // D = diameter of vehicle wheel (_diaWheel converted to _diaWheelP)
    // All length measurements in the same units cancel out
    //
    // http://faculty.salina.k-state.edu/tim/robotics_sg/Control/kinematics/unicycle.html
    // for the modified equation not including the diameter, used below.
    //
    spL = spR = ((float)_ppsMax * vLinear) / 100.0; // convert velocity from % to pps
    SCPRINT("\nSPLR: ", spL);

    spL = spL - ((vAngularR * _lenBaseP) / 2);
    spR = spR + ((vAngularR * _lenBaseP) / 2);

    SCPRINT(" -> pps L:", spL);
    SCPRINT(" R:", spR);

    // put values into the motor setpoint parameters (integers) for running the FSM
    _mData[MLEFT].speed = trunc(spL + 0.5);
    _mData[MRIGHT].speed = trunc(spR + 0.5);

    _mData[MLEFT].state = _mData[MRIGHT].state = S_DRIVE_INIT;
  }
}

void MD_SmartCar2::move(float angL, float angR)
{
  int8_t dirL = 1, dirR = 1;

  SCPRINT("\n** MOVE L:", angL);
  SCPRINT(" R:", angR);

  // set the motor direction
  if (angL < 0.0) { dirL = -1; angL = -angL; }
  if (angR < 0.0) { dirR = -1; angR = -angR; }

  // set the motor % FS setpoint
  _mData[MLEFT].speed = (dirL * abs(_vLinear) * _ppsMax) / 100;
  _mData[MRIGHT].speed = (dirR * abs(_vLinear) * _ppsMax) / 100;
  SCPRINT("\nMove PPS L ", _mData[MLEFT].speed);
  SCPRINT(" R ", _mData[MRIGHT].speed);

  // convert subtended angle into number of pulses
  _mData[MLEFT].pulse = dirL * trunc((angL * _ppr) / (2.0 * PI));
  _mData[MRIGHT].pulse = dirR * trunc((angR * _ppr) / (2.0 * PI));
  SCPRINT("; Pulses L ", _mData[MLEFT].pulse);
  SCPRINT(" R ", _mData[MRIGHT].pulse);

  // finally, set it up for the FSM to execute
  _mData[MLEFT].state = _mData[MRIGHT].state = S_MOVE_INIT;
}

void MD_SmartCar2::spin(int16_t fraction)
// A spin is a symmetrical move() about the center of the rover, 
// so work out the maths on opposing wheel rotations and and then 
// invoke move() with the calculated angles.
{
  int8_t dirL = 1, dirR = 1;

  SCPRINT("\n** SPIN f:", fraction);

  // Work out the wheel directions
  if (fraction < 0.0) dirL = -1;
  if (fraction > 0.0) dirR = -1;
  if (fraction < 0.0) fraction = -fraction; // absolute value

  // Convert fraction into number of encoder pulses. 
  // Both wheels will turn the same number of pulses in opposite directions.
  // 
  // Fractional Circle distance in pulses = PI * base_length_in_pulses * (fraction / 100)
  // Fractional Wheel Distance travelled = PI * wheel_diameter_in_pulses * (Wheel_fraction / 100)
  // 
  // These need to be the same, so equating and simplifying:
  // Wheel_fraction = (base_length_in_pulses * fraction)/wheel_diameter_in_pulses.
  // 
  // Wheel_fraction then converted to wheel rotation angle in radians.
  float angle = 2.0 * PI * (fraction / 100.0) * (_lenBaseP / _diaWheelP);
  SCPRINT(" wheel angle ", angle);

  move(dirL * angle, dirR * angle);
}

void MD_SmartCar2::stop(void)
// Totally halt the vehicle. Set all speeds to 0. 
// The actual state for the motor will change in the FSM when 
// these changes are noticed.
{
  _vLinear = 0;
  _vAngular = 0.0;
  _inSequence = false;

  for (uint8_t i = 0; i < MAX_MOTOR; i++)
  {
    _mData[i].motor->setSpeed(0);
    _mData[i].motor->stop();
  }
}

bool MD_SmartCar2::runActionItem(actionItem_t &ai)
{
  switch (ai.opId)
  {
  case DRIVE:
    SCPRINT("\nSEQ: drive(", ai.parm[0]);
    SCPRINT(", ", ai.parm[1]);
    SCPRINTS(")");
    drive(ai.parm[0], ai.parm[1]);
    _inSequence = true;     // in case speed was 0 and stop() was invoked, we need to re-set this on
    _inAction = false;
    break;

  case MOVE:
    if (!_inAction)
    {
      SCPRINT("\nSEQ: move(", ai.parm[0]);
      SCPRINT(", ", ai.parm[1]);
      SCPRINTS(")");
      move(ai.parm[0], ai.parm[1]);
      _inAction = true;
    }
    else
      _inAction = isRunning();
    break;

  case SPIN:
    if (!_inAction)
    {
      SCPRINT("\nSEQ: spin(", (int16_t)ai.parm[0]);
      SCPRINTS(")");
      spin((int16_t)ai.parm[0]);
      _inAction = true;
    }
    else 
      _inAction = isRunning();
    break;

  case PAUSE:
    if (!_inAction)
    {
      SCPRINT("\nSEQ: pause(", (uint32_t) ai.parm[0]);
      SCPRINTS(")");
      _timeStartSeq = millis();
      _inAction = true;
    }
    else 
      _inAction = (millis() - _timeStartSeq < ai.parm[0]);
    break;

  case STOP:
    SCPRINTS("\nSEQ: stop()");
    stop();
    _inSequence = true;   // stop() sets this false but we need it on for sequences to work!
    _inAction = false;
    break;

  case END:
    SCPRINTS("\nSEQ: end");
    _inSequence = false;
    _inAction = false;
    break;
  }

  return(_inAction);
}

void MD_SmartCar2::startSequence(const actionItem_t* actionList)
{
  if (actionList == nullptr)
    return;

  SCPRINTS("\nSEQ: startSequence PROGMEM");

  // initialise for a new run
  _seqIsConstant = true;
  _uAction.cp = actionList;

  startSeqCommon();
}

void MD_SmartCar2::startSequence(actionItem_t* actionList)
{
  if (actionList == nullptr)
    return;

  SCPRINTS("\nSEQ: startSequence RAM");

  // initialise for a new run
  _seqIsConstant = false;
  _uAction.p = actionList;

  startSeqCommon();
}

void MD_SmartCar2::startSeqCommon(void)
{
  _curActionItem = 0;
  _inSequence = true;
  _inAction = false;

  runSequence();    // do the first step
}

void MD_SmartCar2::runSequence(void)
{
  // If executing a sequence, work with action items
  if (_inSequence)
  {
    if (!_inAction)   // not currently doing anything, load next action item
    {
      if (_seqIsConstant)
        memcpy_P(&_ai, &_uAction.cp[_curActionItem], sizeof(actionItem_t));
      else
        memcpy(&_ai, &_uAction.p[_curActionItem], sizeof(actionItem_t));
      _curActionItem++;
    }

    runActionItem(_ai);   // process current action
  }
}
