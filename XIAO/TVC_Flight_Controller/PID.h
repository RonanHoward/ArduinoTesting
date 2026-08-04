// =====================================================================
//  PID.h  -  a small, generic PID controller (used for pitch and yaw)
//  Advanced file: you normally tune GAINS in config.h, not here.
//
//  Features that matter for flight:
//   - output clamping (never asks for more than the gimbal can give)
//   - integral anti-windup (integral can't wind past the output limit)
//   - derivative on measurement (no "kick" when the setpoint changes)
// =====================================================================
#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
public:
  PID(float kp = 0, float ki = 0, float kd = 0, float outLimit = 1e9f)
    : _kp(kp), _ki(ki), _kd(kd), _outLimit(outLimit),
      _integral(0), _prevMeas(0), _first(true) {}

  void setGains(float kp, float ki, float kd) { _kp = kp; _ki = ki; _kd = kd; }
  void setOutputLimit(float lim) { _outLimit = lim; }

  void reset() { _integral = 0; _prevMeas = 0; _first = true; }

  // Returns the control output for one time step.
  //   setpoint    - the value we want (here: 0 deg tilt = vertical)
  //   measurement - the current value (here: measured tilt angle)
  //   dt          - seconds since the last call
  float update(float setpoint, float measurement, float dt) {
    if (dt <= 0) return 0;
    float error = setpoint - measurement;

    // Proportional
    float p = _kp * error;

    // Integral, with clamping so it can't wind up beyond the output limit
    _integral += error * dt;
    float iTerm = _ki * _integral;
    if (iTerm > _outLimit)      { iTerm =  _outLimit; if (_ki != 0) _integral = iTerm / _ki; }
    else if (iTerm < -_outLimit){ iTerm = -_outLimit; if (_ki != 0) _integral = iTerm / _ki; }

    // Derivative on measurement (not error) to avoid setpoint-change kick
    float d = 0;
    if (!_first) d = -_kd * (measurement - _prevMeas) / dt;
    _prevMeas = measurement;
    _first = false;

    // Sum and clamp
    float out = p + iTerm + d;
    if (out >  _outLimit) out =  _outLimit;
    if (out < -_outLimit) out = -_outLimit;
    return out;
  }

private:
  float _kp, _ki, _kd, _outLimit;
  float _integral, _prevMeas;
  bool  _first;
};

#endif // PID_H
