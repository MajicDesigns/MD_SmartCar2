#pragma once
// Encapsulates vehicle buzzer into one class
//
// - Active piezo buzzer 
// 
// No external dependencies for this class
//

class cBuzzer
{
public:
  // Defines the type of noise allowed
  enum buzzMode_t { SILENT, HEARTBEAT, ALARM, PANIC };

  cBuzzer(uint8_t pin) : _pin(pin), _mode(SILENT), _enable(true) { }

  void begin(void)
  {
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
  }

  void enable(bool e) { _enable = e; if (!e) digitalWrite(_pin, LOW); }

  void setMode(buzzMode_t m)
  {
    _mode = m;
    _state = 0;           // reset to start of sequence
    _timeMark = 0;        // make it happen now!
    digitalWrite(_pin, LOW);    // turn it off for now

    if (_mode == PANIC) enable(true);   // always enable this if PANICKING!
  }

  void run(void)
  {
    if (_enable)
    {
      switch (_mode)
      {
      case SILENT:     /* do nothing */  break;
      case HEARTBEAT:  buzzPulse(50, 2500);  break;
      case ALARM:      buzzAlarm();          break;
      case PANIC:      buzzPulse(2500, 500); break;
      }
    }
  }

private:
  uint8_t _pin;         // pin for the buzzer
  buzzMode_t _mode;     // one of the valid options
  bool _enable;         // runs only when enabled
  uint8_t _state;       // buzzer state
  uint32_t _timeMark;   // millis() time marker

  void buzzPulse(uint16_t onTime, uint16_t offTime)
  {
    switch (_state)
    {
    case 0:   // turn on the pin after TIME_OFF
      if (millis() - _timeMark >= offTime)
      {
        digitalWrite(_pin, HIGH);
        _timeMark = millis();
        _state = 1;
      }
      break;

    case 1: // turn off the pin after TIME_ON
      if (millis() - _timeMark >= onTime)
      {
        digitalWrite(_pin, LOW);
        _timeMark = millis();
        _state = 0;
      }
      break;

    default: _state = 0; break;
    }
  }

  void buzzAlarm(void)
  {
    const uint32_t TIME_ON = 100;    // in ms
    const uint32_t TIME_PAUSE = 500;// in ms
    const uint32_t TIME_OFF = 2000; // in ms
    static uint8_t count = 0;

    switch (_state)
    {
    case 0:   // turn on the pin after TIME_PAUSE
      if (millis() - _timeMark >= TIME_PAUSE)
      {
        digitalWrite(_pin, HIGH);
        _timeMark = millis();
        count++;
        _state = 1;
      }
      break;

    case 1: // turn off the pin after TIME_ON
      if (millis() - _timeMark >= TIME_ON)
      {
        digitalWrite(_pin, LOW);
        _timeMark = millis();
        _state = (count == 2) ? 2 : 0;
      }
      break;

    case 2: // turn wait for TIME_OFF
      if (millis() - _timeMark >= TIME_OFF)
      {
        _timeMark = millis();
        count = 0;
        _state = 0;
      }
      break;

    default: _state = 0; break;
    }
  }
};