/*
  nRF24L01 Instrument Station
  PolledInterval.hpp

  D C Potts 2021
  School of Engineering
  University of Liverpool
*/

#ifndef IS_POLLEDINTERVAL_HPP
#define IS_POLLEDINTERVAL_HPP

#include <stdint.h>

#include "Arduino.h"

typedef void (* intervalFunc_t)(uintptr_t);

class PolledInterval
{
public:
  PolledInterval(intervalFunc_t func, uint32_t interval, uintptr_t context = 0)
  {
    _enabled = false;
    _interval = interval;
    _context = context;
    _func = func;
  };
  ~PolledInterval() {};

  void setContext(uintptr_t context)
  {
    _context = context;
  };

  void enable()
  {
    _enabled = true;
  };

  void disable()
  {
    _enabled = false;
  };

  void setEnabled(bool enable)
  {
    _enabled = enable;
  };

  void run()
  {
    if (!_enabled) {
      return;
    }
    uint32_t currentTime = millis();
    if ((currentTime - _previousInterval) >= _interval) {
      _previousInterval = currentTime;
      _func(_context);
    }
  };

private:
  bool _enabled;
  uintptr_t _context;
  uint32_t _interval;
  uint32_t _previousInterval;
  intervalFunc_t _func;

};

#endif
