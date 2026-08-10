// =====================================================================
//  FlightController.h  -  the flight brain (state machine)
//  Advanced file. Ties the sensors, estimator, PIDs, gimbal, and
//  parachute together and sequences the flight:
//
//    PREFLIGHT --(launch pin high)--> ASCENT
//    ASCENT    --(baro descent OR backup timer)--> DESCENT
//    DESCENT   --(chute out)--> LANDED
//    (any bring-up failure) -> ERROR (safe: centered gimbal, chute stowed)
//
//  Normal tuning lives in config.h. The one thing here you might swap is
//  detectLaunch(), which will become an accelerometer trigger later.
// =====================================================================
#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "lsm6dsv.h"
#include "lps22hb.h"
#include "AttitudeEstimator.h"
#include "PID.h"
#include "TVCMount.h"
#include "Parachute.h"
#include "Telemetrylog.h"

enum FlightState { STATE_PREFLIGHT, STATE_ASCENT, STATE_DESCENT, STATE_LANDED, STATE_ERROR };

class FlightController {
public:
  FlightController()
    : _state(STATE_PREFLIGHT),
      _pitchPID(PITCH_KP, PITCH_KI, PITCH_KD, PID_OUTPUT_LIMIT_DEG),
      _rollPID(ROLL_KP, ROLL_KI, ROLL_KD, PID_OUTPUT_LIMIT_DEG),
      _launchTime(0), _lastBaro(0) {}

  // One-time startup. Call from setup().
  void begin() {
    pinMode(LAUNCH_PIN, INPUT);

    if (!bringUpSensors()) { _state = STATE_ERROR; return; }

    _tvc.begin();       // attach + center gimbal servos
    _chute.begin();     // attach + stow parachute servo

    if (!_est.calibrate(GYRO_BIAS_SAMPLES)) { _state = STATE_ERROR; return; }

    float p, t;         // capture pad pressure as the altitude zero
    if (readBaroNow(p, t)) _chute.setGroundPressure(p);

    if (!_telemetryLog.begin()) { _state = STATE_ERROR; return; }

    _state = STATE_PREFLIGHT;
  }

  // Call every loop().
  void update() {
    switch (_state) {
      case STATE_PREFLIGHT:
        preflight();
        _telemetryLog.poll();
        break;
      case STATE_ASCENT:    ascent();     break;
      case STATE_DESCENT:   descent();    break;
      case STATE_LANDED:
        landed();
        _telemetryLog.poll();
        break;
      case STATE_ERROR:     errorState(); break;
    }
    
  }

  FlightState state() { return _state; }

private:
  FlightState _state;
  AttitudeEstimator _est;
  PID _pitchPID, _rollPID;
  TVCMount _tvc;
  Parachute _chute;
  TelemetryLog _telemetryLog;
  unsigned long _launchTime, _lastBaro, _lastTelemLog;
  float _pitchCmd = 0.0f;
  float _rollCmd = 0.0f;


  // ---------------- states ----------------
  void preflight() {
    _tvc.center();                       // hold gimbal straight on the pad
    // if (detectLaunch()) {
    if (true) {
      _launchTime = millis();
      _est.resetOrientation();           // launch attitude becomes "vertical"
      _pitchPID.reset();
      _rollPID.reset();
      _state = STATE_ASCENT;
      // tone(SPEAKER_PIN, SPEAKER_TONE_FREQ, 400);
    }
  }

  void ascent() {
    // Control loop: on each new gyro sample, integrate then run PID -> gimbal.
    if (_est.update()) {
      float dt = _est.lastDt();
      // Setpoint 0 = keep the launch attitude (stay vertical).
      _pitchCmd = _pitchPID.update(0.0f, PITCH_AXIS_SIGN * _est.pitch(), dt);
      _rollCmd  = _rollPID.update (0.0f, ROLL_AXIS_SIGN   * _est.roll(),  dt);
      _tvc.command(_pitchCmd, _rollCmd);
    }

    serviceBaro();  // ~25 Hz altitude/apogee tracking

    // Automatically advance to next flight state after set time (FOR CURRENT FLIGHTS)
    if (millis() - _launchTime >= DEBUG_ASCENT_TIME_S * 1000) {
      _state = STATE_DESCENT;
    }

    // if (_chute.shouldDeploy(millis() - _launchTime)) {
    //   _state = STATE_DESCENT;
    // }

    telemetry();
  }

  void descent() {
    // _chute.deploy();
    _tvc.center();
    _tvc.detach();                  // relax the gimbal under the chute
    _state = STATE_LANDED;
  }

  void landed() {
    _telemetryLog.commit();
  }

  void errorState() {
    // Fail safe: gimbal centered, chute stowed, report periodically.
    static unsigned long last = 0;
    if (millis() - last > 1000) {
      last = millis();
      Serial.println("ERROR: sensor bring-up failed. Check wiring/pull-ups.");
    }
  }

  // ------------- launch detection (swap for accelerometer later) -------------
  bool detectLaunch() {
    return analogRead(LAUNCH_PIN) > LAUNCH_THRESHOLD_COUNTS;
  }

  // ------------- barometer helpers -------------
  bool readBaroNow(float &p, float &t) {
    if (BARO.pressureAvailable()) { BARO.readPressure(p); BARO.readTemperature(t); return true; }
    return false;
  }
  void serviceBaro() {
    if (millis() - _lastBaro >= 40) {    // ~25 Hz, matches the baro ODR
      _lastBaro = millis();
      if (BARO.pressureAvailable()) {
        float p;
        BARO.readPressure(p);
        _chute.updateAltitude(p);
      }
    }
  }

  // ------------- telemetry -------------
  void telemetry() {
#if TELEMETRY_ENABLED
    unsigned long telem_loop_now = millis();
    if (telem_loop_now - _lastTelemLog >= 1000 / TELEMETRY_LOG_FREQ_HZ) {
      _telemetryLog.log(_est.roll(), _est.pitch(), _rollCmd, _pitchCmd, _chute.altitude());
      _lastTelemLog = telem_loop_now;
    }
#endif
  }

  // ------------- sensor bring-up (bus recovery + software reset + retry) -----
  bool bringUpSensors() {
    bool ok = false;
    for (int attempt = 1; attempt <= 3 && !ok; attempt++) {
      i2cBusRecovery(SDA, SCL);          // free a stuck slave at the bus level
      Wire.begin();                      // (re)claim the pins for I2C
      Wire.setClock(400000);
      ok = IMU.begin() && BARO.begin();  // each does its own software reset
      if (!ok) delay(10);
    }
    return ok;
  }

  // Clock a stuck I2C slave free, then STOP. Call before Wire.begin().
  static void i2cBusRecovery(uint8_t sdaPin, uint8_t sclPin) {
    pinMode(sclPin, INPUT_PULLUP);
    pinMode(sdaPin, INPUT_PULLUP);
    delayMicroseconds(10);
    for (int i = 0; i < 9 && digitalRead(sdaPin) == LOW; i++) {
      pinMode(sclPin, OUTPUT); digitalWrite(sclPin, LOW); delayMicroseconds(5);
      pinMode(sclPin, INPUT_PULLUP);                      delayMicroseconds(5);
    }
    pinMode(sdaPin, OUTPUT); digitalWrite(sdaPin, LOW); delayMicroseconds(5);
    pinMode(sclPin, INPUT_PULLUP);                      delayMicroseconds(5);
    pinMode(sdaPin, INPUT_PULLUP);                      delayMicroseconds(5);
  }
};

#endif // FLIGHT_CONTROLLER_H
