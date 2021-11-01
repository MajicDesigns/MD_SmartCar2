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

### Measure Physical Constants

An application using the library needs to pass a few physical constants to the 
MD_SmartCar2::begin() method that allows the library to configure vehicle control
parameters.

Note that the units of length are specified in millimeters. However, alternative 
units (eg inches or centimeters) may be used __AS LONG AS ALL LENGTHS ARE 
SPECIFIED IN THE SAME UNITS__, as the units all cancel out in the calculations.

Two physical constants need to be directly measured from the vehicle:
- The __wheel diameter__ in mm. This is measured to the outer edge of the wheel tire.
- The __base length__ (distance between wheel centers) in mm. Inside edge of one wheel 
to the inside edge of the other wheel.

These two constants are shown as _2r_ and _B_ in the figure below.

![SmartCar2 Unicycle](SmartCar2_Unicycle.png "SmartCar2 Distances Measured")

- The number of __encoder pulses per wheel revolution__. This is a given constant that
depends on the stepper motor/gearbox combination. 

- The __maximum number of encoder pulses per second__ defining top speed (100% velocity).
This is determined by experimentation. A smaller number sets the control top speed to the 
specified fraction of the actual top speed.

### Confirming Motor Setup

The first part of setup is to use the __MotorTest__ example sketch to ensure 
that the motors are turning in the correct direction and the encoders are 
working correctly.

MotorTest allows control of the vehicle motors using the Serial Monitor through 
the command line interface to invoke test the functions (type ? for help text 
listing the functions).

Independently commanding the motors to move in a Forward and Reverse direction 
provides confirmation that they are wired correctly. If they rotate the wrong 
(opposite) direction, simply reverse pairs of wires (or their I/O pin numbers)
between processor I/O pins and the speed controller.

The next step is to use the __MotorTest__ example sketch to test coordinated
motion of the motors y invoking the library MD_SmartCar2::drive(), 
MD_SmartCar2::move() and MD_SmartCar2::spin() methods.

The final phase, using the __Control_Test__ example, is about testing the vehicle 
'in real life' as it moves about on the floor. The dynamics in this situation are 
likely to be different from the previous bench testing previously.

The __Control_Test__ example sketch is paired with the related App Inventor 2 (AI2, 
see http://ai2.appinventor.mit.edu/) "Control Test" application found in the example 
sketch folder. The AI2 application provides a GUI front end for commands through a 
Bluetooth interface. The same commands could be issued from the Serial Monitor 
(or other Terminal program) connected through a Bluetooth serial port.

The AI2 application has a main menu leading to a displays for controlling 
drive() and move().

\page pageControlModel Unicycle Control Model

Working out the displacement and velocities of each wheel on a differential 
drive robot can be messy.

The _unicycle model_ of an autonomous robot is a kinematic model that allows 
modeling the movement of the vehicle as if it was a unicycle, using a linear 
velocity vector (V) and a rotational velocity (&omega;) about a point within 
the vehicle. Taking this point to be midway between axis joining the 2 wheels 
simplifies the calculation.

![SmartCar2 Movement Transform](SmartCar2_Transform.png "SmartCar2 Movement Transform")

We can derive equations that translate between the unicycle model and our wheel 
velocities. Steering requires each of the independent wheels to rotated at 
different speeds (V<sub>L</sub> and V<sub>R</sub> for the left and right side) 
to travel the equivalent unicycle path.

So specifying a movement path using this abstraction becomes much easier as we 
need to just specify "How fast do we want to move forward and how fast do we 
want to turn", letting the mathematics work out the wheel rotations.

![SmartCar2 Unicycle](SmartCar2_Unicycle.png "SmartCar2 Unicycle Model")

V and &omega; are transformed into independent motor speeds for the left and right 
motor (V<sub>L</sub>, V<sub>R</sub>) using the following formulas:
- V<sub>L</sub> = (2V + &omega;B) / 2r
- V<sub>R</sub> = (2V - &omega;B) / 2r

where B is the vehicle base length (ie, the distance between the wheel centerlines) 
and r is the radius of the wheel.

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

|          ActionId_t | Comment          | Parameter 0     | Parameter 1
|--------------------:|:-----------------|:----------------|:------------
| MD_SmartCar2::DRIVE | executes drive() | Linear Velocity | Angular Velocity
|  MD_SmartCar2::MOVE | executes move()  | Left rotate     | Right rotate
|  MD_SmartCar2::SPIN | executes spin()  | Spin percentage | Not used
| MD_SmartCar2::PAUSE | executes pause   | Milliseconds    | Not used
|  MD_SmartCar2::STOP | executes stop()  | Not used        | Not used
|   MD_SmartCar2::END | marks seq end    | Not used        | Not used

*/

MD_SmartCar2::MD_SmartCar2(MD_Stepper *ml, MD_Stepper *mr)
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

  // begin and intialize the motors
  for (uint8_t i = 0; i < MAX_MOTOR; i++)
  {
    _mData[i].motor->begin();
    _mData[i].motor->setStepMode(MD_Stepper::HALF);
    _mData[i].motor->setMotorLockTime(0);
    _mData[i].speedCV = _mData[i].speedSP = 0;
  }

  // Set up default environment
  setVehicleParameters(ppr, maxSpeed, dWheel, lBase);
  setAccelProfile(DEF_ACCEL_TIME, DEF_ACCEL_STEPS);
  stop();    // initialize to all stop

  return(b);
}

void MD_SmartCar2::run(void)
// run the FSM to manage motor functions
{
#define MOVE_DETAIL 0    // enable detailed move debug
#define RUN_DETAIL  0    // enable detailed run debug

  bool firstPass = true;

  // run the sequence to set up a command if we are currently in that mode
  if (_inSequence)
    runSequence();

  // loop through all the motors doing whatever in each state
  for (uint8_t motor = 0; motor < MAX_MOTOR; motor++)   // right[1] side stops
//  for (int8_t motor = MAX_MOTOR-1; motor >= 0; motor--) // left[0] side stops
  {
    // always let the motors do something
    _mData[motor].motor->run();

    // now deal with any changes to the current motor driving conditions
    switch (_mData[motor].state)
    {
    case S_IDLE: 
      if (_mData[motor].motor->isBusy())
      {
        SCPRINT("\n?? IDLE with busy #", motor);
        _mData[motor].motor->stop();
      }
      break;     // do nothing

    // --- FREE RUNNING
    case S_ACCEL_INIT:    // initialize acceleration sequence
      SCPRINT("\n>>ACCEL_INIT #", motor);
      // calculate the step size for acceleration
      _mData[motor].pulse = (_mData[motor].speedSP - _mData[motor].speedCV) / _accelSteps;
      if (_mData[motor].pulse == 0) _mData[motor].pulse = 1;

      _mData[motor].speedCV += _mData[motor].pulse;
      _mData[motor].motor->setVelocity(_mData[motor].speedCV);
      _mData[motor].motor->start();

      _mData[motor].timeMark = 0;
      _mData[motor].state = S_ACCEL;
      break;

    case S_ACCEL:         // accelerate to set speed
      if (millis() - _mData[motor].timeMark >= _accelTime)
      {
        // see if we are done or need to work out velocity for next time
        if (_mData[motor].speedCV >= _mData[motor].speedSP)
        {
          SCPRINT("\n>>ACCEL to DRIVE_RUN #", motor);
          _mData[motor].state = S_DRIVE_RUN;
        }
        else
        {
          _mData[motor].speedCV += _mData[motor].pulse;
          if (_mData[motor].speedCV > _mData[motor].speedSP)
            _mData[motor].speedCV = _mData[motor].speedSP;

#if RUN_DETAIL
          SCPRINT("\n>>ACCEL new speed #", motor);
          SCPRINT(" @ ", _mData[motor].speedCV);
          SCPRINT(" of ", _mData[motor].speedSP);
#endif
        }

        _mData[motor].motor->setVelocity(_mData[motor].speedCV);
        _mData[motor].timeMark = millis();
      }
      break;

    case S_DRIVE_RUN:     // run at constant speed
      // if motor is not running, deal with it
      if (!_mData[motor].motor->isBusy())
      {
        SCPRINT("\n>>DRIVE_RUN to IDLE #", motor);
        _mData[motor].speedCV = 0;
        _mData[motor].state = S_IDLE;
      }
      else
      {
        // check if the speed has changed and change state accordingly
        if (_mData[motor].speedCV < _mData[motor].speedSP)
        {
          SCPRINT("\n>>DRIVE_RUN to ACCEL_INIT #", motor);
          _mData[motor].state = S_ACCEL_INIT;
        }
        else if (_mData[motor].speedCV > _mData[motor].speedSP)
        {
          SCPRINT("\n>>DRIVE_RUN to DECEL_INIT #", motor);
          _mData[motor].state = S_DECEL_INIT;
        }
      }
      break;

    case S_DECEL_INIT:
      SCPRINT("\n>>DECEL_INIT #", motor);
      // calculate the step size for acceleration
      _mData[motor].pulse = (_mData[motor].speedCV - _mData[motor].speedSP) / _accelSteps;
      if (_mData[motor].pulse == 0) _mData[motor].pulse = 1;

      _mData[motor].speedCV -= _mData[motor].pulse;

      _mData[motor].timeMark = 0;
      _mData[motor].state = S_DECEL;
      break;

    case S_DECEL:
      if (millis() - _mData[motor].timeMark >= _accelTime)
      {
        _mData[motor].motor->setVelocity(_mData[motor].speedCV);
        _mData[motor].timeMark = millis();

        // now check if we have reached the target speed
        if (_mData[motor].speedCV <= _mData[motor].speedSP)
        {
          // reached speed is zero, so shut the motor down
          if (_mData[motor].speedSP == 0)
          {
            SCPRINT("\n>>DECEL to IDLE #", motor);
            _mData[motor].state = S_IDLE;
          }
          else  // it was just another (lower) speed, so keep running at this speed
          {
            SCPRINT("\n>>DECEL to DRIVE_RUN #", motor);
            _mData[motor].state = S_DRIVE_RUN;
          }
        }
        else  // work out the speed for the next pass
        {
          _mData[motor].speedCV -= _mData[motor].pulse;
          if (_mData[motor].speedCV < _mData[motor].speedSP)
            _mData[motor].speedCV = _mData[motor].speedSP;

#if RUN_DETAIL
          SCPRINT("\n>>DECEL new speed #", motor);
          SCPRINT(" @ ", _mData[motor].speedCV);
          SCPRINT(" of ", _mData[motor].speedSP);
#endif
        }
      }
      break;

    // --- PRECISION STEP BASED MOVES
    case S_MOVE_INIT:
      SCPRINT("\n>>MOVE_INIT #", motor);
      _mData[motor].motor->setVelocity(_mData[motor].speedSP);
      _mData[motor].motor->move(_mData[motor].pulse);
      _mData[motor].motor->start();
      _mData[motor].state = S_MOVE_RUN;
      break;

    case S_MOVE_RUN:
      if (firstPass)
      {
        firstPass = false;
#if MOVE_DETAIL
        SCPRINTS("\nMOVE");
#endif
      }
#if MOVE_DETAIL
      else
      {
        SCPRINTS(",");
        SCPRINT(" [", motor);
        SCPRINT("] ", _mData[motor].motor->moveToGo());
        SCPRINT("/", _mData[motor].pulse);
      }
#endif
        
      // check for ending conditions
      if (!_mData[motor].motor->isBusy())
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

  SCPRINT("\n** DRIVE v:", vLinear);
  SCPRINT(" a:", vAngularR);

  // don't do anything if parameters don't change
  //if (vLinear == _vLinear && vAngularR == _vAngular)
  //  return;

  // sanitize input
  if (vLinear < -100) vLinear = -100;
  if (vLinear > 100) vLinear = 100;
  if (vAngularR < -PI/2) vAngularR = -PI/2;
  if (vAngularR > PI/2)  vAngularR = PI/2;

  // save these for reporting/other use
  _vLinear = vLinear;
  _vAngular = vAngularR;
    
  // set up for calculations
  vAngularR = -vAngularR;  // reverse the library convention for calcs

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
  _mData[MLEFT].speedSP = trunc(spL + 0.5);
  _mData[MRIGHT].speedSP = trunc(spR + 0.5);

  // work out if we need to kick off movement
  if (_mData[MLEFT].state != S_DRIVE_RUN) _mData[MLEFT].state = S_ACCEL_INIT;
  if (_mData[MRIGHT].state != S_DRIVE_RUN) _mData[MRIGHT].state = S_ACCEL_INIT;
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
  if (_vLinear == 0) _vLinear = MOVE_SPEED;
  _mData[MLEFT].speedSP = dirL * ((_vLinear * _ppsMax) / 100);
  _mData[MRIGHT].speedSP = dirR * ((_vLinear * _ppsMax) / 100);
  SCPRINT("\nMove PPS L ", _mData[MLEFT].speedSP);
  SCPRINT(" R ", _mData[MRIGHT].speedSP);

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

void MD_SmartCar2::stop(bool insideSequence)
// Immediately halt the vehicle. 
// The actual state for the motor will change in the run() FSM when 
// these changes are noticed.
{
  _inSequence = insideSequence;

  for (uint8_t i = 0; i < MAX_MOTOR; i++)
  {
    _mData[i].speedCV = 0;
    _mData[i].state = S_IDLE;
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
    {
      SCPRINT("\nSEQ: T ", millis()); // -(uint32_t)ai.parm[0]);
      _inAction = (millis() - _timeStartSeq < ai.parm[0]);
      if (!_inAction) SCPRINTS("\nSEQ: pause end");
    }
    break;

  case STOP:
    SCPRINTS("\nSEQ: stop()");
    stop(_inSequence);
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

  // initialize for a new run
  _seqIsConstant = true;
  _uAction.cp = actionList;

  startSeqCommon();
}

void MD_SmartCar2::startSequence(actionItem_t* actionList)
{
  if (actionList == nullptr)
    return;

  SCPRINTS("\nSEQ: startSequence RAM");

  // initialize for a new run
  _seqIsConstant = false;
  _uAction.p = actionList;

  startSeqCommon();
}

void MD_SmartCar2::stopSequence(void)
{
  _inSequence = false;
  _inAction = false;
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
