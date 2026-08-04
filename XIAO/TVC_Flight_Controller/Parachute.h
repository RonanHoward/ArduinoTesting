// =====================================================================
//  Parachute.h  -  apogee detection + release servo
//  Advanced file. It is fed barometric pressure samples, tracks the
//  highest altitude reached, and decides when to deploy:
//    - primary:  altitude has fallen DESCENT_DROP_M below apogee,
//                confirmed over DESCENT_CONFIRM_SAMPLES samples
//    - failsafe: a backup timer fires no matter what
//  Tune the thresholds in config.h.
// =====================================================================
#ifndef PARACHUTE_H
#define PARACHUTE_H

#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include "config.h"

class Parachute {
public:
  void begin() {
    _servo.attach(SERVO_CHUTE_PIN);
    stow();
    _deployed = false;
    _descentCount = 0;
    _groundP = 1013.25f;      // replaced by setGroundPressure() on the pad
    _alt = 0;
    _maxAlt = 0;
  }

  void stow() { _servo.writeMicroseconds(CHUTE_STOWED_US); }

  // Capture pad pressure as the altitude reference (call before launch).
  void setGroundPressure(float hPa) { _groundP = hPa; _maxAlt = 0; _alt = 0; }

  // Feed a new pressure sample (hPa). Updates altitude and apogee.
  void updateAltitude(float pressure_hPa) {
    // Barometric formula, altitude above the pad in meters.
    _alt = 44330.0f * (1.0f - powf(pressure_hPa / _groundP, 0.1902949f));
    if (_alt > _maxAlt) _maxAlt = _alt;
  }

  // Decide whether to deploy now. msSinceLaunch drives the failsafe timer.
  bool shouldDeploy(unsigned long msSinceLaunch) {
    if (_deployed) return false;

    if (msSinceLaunch >= BACKUP_APOGEE_TIME_MS) return true;   // failsafe

    if (_maxAlt - _alt >= DESCENT_DROP_M) {                    // primary
      if (++_descentCount >= DESCENT_CONFIRM_SAMPLES) return true;
    } else {
      _descentCount = 0;
    }
    return false;
  }

  void deploy() { _servo.writeMicroseconds(CHUTE_DEPLOYED_US); _deployed = true; }

  bool  deployed() { return _deployed; }
  float altitude() { return _alt; }
  float apogee()   { return _maxAlt; }

private:
  Servo _servo;
  float _groundP, _alt, _maxAlt;
  int   _descentCount;
  bool  _deployed;
};

#endif // PARACHUTE_H
