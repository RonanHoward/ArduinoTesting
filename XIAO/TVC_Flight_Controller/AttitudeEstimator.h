// =====================================================================
//  AttitudeEstimator.h  -  turns gyro data into a tilt estimate
//  Advanced file. Wraps the gyro driver (IMU) and does:
//    - bias calibration on the pad
//    - quaternion integration of angular rate
//    - readout as pitch / yaw / roll in degrees, relative to the
//      orientation captured at launch (so 0 = "as it left the pad")
//
//  Gyro-only: this DRIFTS slowly with no absolute reference. That is
//  acceptable for the seconds-long powered/coast phase of a flight.
// =====================================================================
#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include <Arduino.h>
#include <math.h>
#include "lsm6dsv.h"

#ifndef DEG2RAD
#define DEG2RAD 0.01745329252f
#endif

class AttitudeEstimator {
public:
  AttitudeEstimator() {
    resetOrientation();
    _bx = _by = _bz = 0; _dt = 0;
  }

  // Average N still samples to measure gyro bias. Rocket must be still.
  // Returns false if it times out (sensor not responding).
  bool calibrate(int samples) {
    double sx = 0, sy = 0, sz = 0;
    int c = 0;
    unsigned long t0 = millis();
    while (c < samples) {
      if (IMU.gyroscopeAvailable()) {
        float gx, gy, gz;
        IMU.readGyroscope(gx, gy, gz);
        sx += gx; sy += gy; sz += gz; c++;
      }
      if (millis() - t0 > 5000) return false;
    }
    _bx = sx / samples; _by = sy / samples; _bz = sz / samples;
    return true;
  }

  // Set current orientation as the "zero" reference (call at launch).
  void resetOrientation() {
    _q[0] = 1; _q[1] = _q[2] = _q[3] = 0;
    _tPrev = micros();
  }

  // Non-blocking: integrate one sample if a new one is ready.
  // Returns true if the estimate advanced this call.
  bool update() {
    if (!IMU.gyroscopeAvailable()) return false;
    float gx, gy, gz;
    IMU.readGyroscope(gx, gy, gz);              // dps

    unsigned long now = micros();
    _dt = (now - _tPrev) * 1e-6f;
    _tPrev = now;

    float wx = (gx - _bx) * DEG2RAD;            // bias-corrected, rad/s
    float wy = (gy - _by) * DEG2RAD;
    float wz = (gz - _bz) * DEG2RAD;
    integrate(wx, wy, wz, _dt);
    return true;
  }

  float lastDt() { return _dt; }

  // Euler angles (degrees) relative to the last resetOrientation().
  float roll()  { return atan2f(2*(_q[0]*_q[1] + _q[2]*_q[3]),
                                1 - 2*(_q[1]*_q[1] + _q[2]*_q[2])) / DEG2RAD; }
  float pitch() { float s = 2*(_q[0]*_q[2] - _q[3]*_q[1]);
                  s = (fabsf(s) >= 1.0f) ? copysignf(PI/2, s) : asinf(s);
                  return s / DEG2RAD; }
  float yaw()   { return atan2f(2*(_q[0]*_q[3] + _q[1]*_q[2]),
                                1 - 2*(_q[2]*_q[2] + _q[3]*_q[3])) / DEG2RAD; }

private:
  float _q[4];
  float _bx, _by, _bz;         // gyro bias (dps)
  unsigned long _tPrev;
  float _dt;

  void integrate(float wx, float wy, float wz, float dt) {
    float wmag = sqrtf(wx*wx + wy*wy + wz*wz);
    float d0, d1, d2, d3;
    if (wmag > 1e-8f) {
      float half = 0.5f * wmag * dt;
      float s = sinf(half) / wmag;
      d0 = cosf(half); d1 = wx*s; d2 = wy*s; d3 = wz*s;
    } else {
      d0 = 1; d1 = d2 = d3 = 0;
    }
    float nw = _q[0]*d0 - _q[1]*d1 - _q[2]*d2 - _q[3]*d3;
    float nx = _q[0]*d1 + _q[1]*d0 + _q[2]*d3 - _q[3]*d2;
    float ny = _q[0]*d2 - _q[1]*d3 + _q[2]*d0 + _q[3]*d1;
    float nz = _q[0]*d3 + _q[1]*d2 - _q[2]*d1 + _q[3]*d0;
    float inv = 1.0f / sqrtf(nw*nw + nx*nx + ny*ny + nz*nz);
    _q[0] = nw*inv; _q[1] = nx*inv; _q[2] = ny*inv; _q[3] = nz*inv;
  }
};

#endif // ATTITUDE_ESTIMATOR_H
