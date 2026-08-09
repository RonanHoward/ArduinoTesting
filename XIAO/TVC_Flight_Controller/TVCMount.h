// =====================================================================
//  TVCMount.h  -  drives the two gimbal servos
//  Advanced file. Converts a desired PITCH and YAW motor-deflection
//  angle (from the PID) into servo pulse widths, applying:
//    - the mechanical deflection limit (clamp)
//    - the per-axis direction sign (from config.h)
//    - the center/trim and microseconds-per-degree (from config.h)
//  Tune the numbers in config.h, not here.
// =====================================================================
#ifndef TVC_MOUNT_H
#define TVC_MOUNT_H

#include <Arduino.h>
#include <Servo.h>
#include "config.h"

class TVCMount {
public:
  void begin() {
    _pitch.attach(SERVO_PITCH_PIN);
    _roll.attach(SERVO_ROLL_PIN);
    center();
  }

  void center() { command(0.0f, 0.0f); }

  // pitchDeg / yawDeg: desired motor deflection angles from the PID.
  void command(float pitchDeg, float rollDeg) {
    pitchDeg = clampDeflect(pitchDeg);
    rollDeg   = clampDeflect(rollDeg);

    int pDeg = SERVO_PITCH_CENTER_DEG + (int)(SERVO_PITCH_SIGN * pitchDeg * 2.5);
    int yDeg = SERVO_ROLL_CENTER_DEG  + (int)(SERVO_ROLL_SIGN  * rollDeg  * 2  );

    _pitch.write(pDeg);
    _roll.write(yDeg);
  }

  // Relax the servos (e.g. after the chute is out).
  void detach() { _pitch.detach(); _roll.detach(); }

private:
  Servo _pitch, _roll;

  float clampDeflect(float a) {
    if (a >  TVC_MAX_DEFLECT_DEG) return  TVC_MAX_DEFLECT_DEG;
    if (a < -TVC_MAX_DEFLECT_DEG) return -TVC_MAX_DEFLECT_DEG;
    return a;
  }
};

#endif // TVC_MOUNT_H
