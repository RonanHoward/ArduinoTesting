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
    _yaw.attach(SERVO_YAW_PIN);
    center();
  }

  void center() { command(0.0f, 0.0f); }

  // pitchDeg / yawDeg: desired motor deflection angles from the PID.
  void command(float pitchDeg, float yawDeg) {
    pitchDeg = clampDeflect(pitchDeg);
    yawDeg   = clampDeflect(yawDeg);

    int pUs = SERVO_PITCH_CENTER_US + (int)(SERVO_PITCH_SIGN * pitchDeg * SERVO_US_PER_DEG);
    int yUs = SERVO_YAW_CENTER_US   + (int)(SERVO_YAW_SIGN   * yawDeg   * SERVO_US_PER_DEG);

    _pitch.writeMicroseconds(pUs);
    _yaw.writeMicroseconds(yUs);
  }

  // Relax the servos (e.g. after the chute is out).
  void detach() { _pitch.detach(); _yaw.detach(); }

private:
  Servo _pitch, _yaw;

  float clampDeflect(float a) {
    if (a >  TVC_MAX_DEFLECT_DEG) return  TVC_MAX_DEFLECT_DEG;
    if (a < -TVC_MAX_DEFLECT_DEG) return -TVC_MAX_DEFLECT_DEG;
    return a;
  }
};

#endif // TVC_MOUNT_H
