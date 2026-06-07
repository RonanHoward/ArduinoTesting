#include "Arduino_BMI270_BMM150.h"
#include <math.h>

// ---------------------------------------------------------------------------
// CONFIGURATION
// ---------------------------------------------------------------------------

// Gyro bias (deg/s), already measured, in BOARD axes
const float BIAS_X = -0.08173f;
const float BIAS_Y = -0.03525f;
const float BIAS_Z = -0.06125f;

// Optional gyro scale-factor correction (1.0 = none)
const float SCALE_X = 1.0f;
const float SCALE_Y = 1.0f;
const float SCALE_Z = 1.0f;

// Reporting signs (do not affect the math). Flip if the lean direction comes
// out opposite to the convention you want
const float PITCH_SIGN = 1.0f;
const float ROLL_SIGN  = 1.0f;

// Output rate. Integration always runs at the full IMU sample rate; only the
// Serial print is throttled so it never starves the read loop.
const unsigned long PRINT_PERIOD_MS = 50;   // 20 Hz

// ---------------------------------------------------------------------------
// STATE
// ---------------------------------------------------------------------------

// Attitude quaternion (body -> launch frame), [w, x, y, z]. Identity = vertical
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

unsigned long lastMicros = 0;
unsigned long lastPrint  = 0;

static const float DEG2RAD = 0.017453292519943295f;
static const float RAD2DEG = 57.29577951308232f;

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  if (!IMU.begin()) {
    while (1) { Serial.println("IMU init failed"); delay(1000); }
  }

  // Launch-detect instant: rocket vertical & stationary -> identity quaternion
  q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;

  lastMicros = micros();
  lastPrint  = millis();
}

// ---------------------------------------------------------------------------
void loop() {
  if (IMU.gyroscopeAvailable()) {
    float gx, gy, gz;                 // deg/s, BOARD axes
    IMU.readGyroscope(gx, gy, gz);

    // Real elapsed time for the step (more accurate than a fixed dt)
    unsigned long now = micros();
    float dt = (now - lastMicros) * 1.0e-6f;
    lastMicros = now;

    // Skip the first sample and any abnormally large step
    if (dt > 0.0f && dt < 0.1f) {

      // Remove bias and scale error.
      gx = (gx - BIAS_X) * SCALE_X;
      gy = (gy - BIAS_Y) * SCALE_Y;
      gz = (gz - BIAS_Z) * SCALE_Z;

      // Board axes -> body axes (top face toward nose => identity, Z = long axis)
      float wx = gx * DEG2RAD;        // rad/s, body X (transverse)
      float wy = gy * DEG2RAD;        // rad/s, body Y (transverse)
      float wz = gz * DEG2RAD;        // rad/s, body Z (long axis / spin)

      // Integrate with the exact exponential map. A body rate that is constant
      // over the step (e.g. steady spin) is integrated exactly at any dt;
      // residual error comes only from the rate changing within a step
      float dthx = wx * dt;
      float dthy = wy * dt;
      float dthz = wz * dt;
      float theta = sqrtf(dthx*dthx + dthy*dthy + dthz*dthz);

      float dw, dx, dy, dz;           // incremental rotation quaternion
      if (theta > 1.0e-8f) {
        float half = 0.5f * theta;
        float s = sinf(half) / theta;
        dw = cosf(half);
        dx = dthx * s;
        dy = dthy * s;
        dz = dthz * s;
      } else {                        // small-angle limit (avoids div-by-zero)
        dw = 1.0f;
        dx = 0.5f * dthx;
        dy = 0.5f * dthy;
        dz = 0.5f * dthz;
      }

      // q = q (x) dq   (Hamilton product; body-frame rate update)
      float nq0 = q0*dw - q1*dx - q2*dy - q3*dz;
      float nq1 = q0*dx + q1*dw + q2*dz - q3*dy;
      float nq2 = q0*dy - q1*dz + q2*dw + q3*dx;
      float nq3 = q0*dz + q1*dy - q2*dx + q3*dw;

      // Renormalise.
      float inv = 1.0f / sqrtf(nq0*nq0 + nq1*nq1 + nq2*nq2 + nq3*nq3);
      q0 = nq0*inv; q1 = nq1*inv; q2 = nq2*inv; q3 = nq3*inv;
    }
  }

  // -------- Output (throttled) -----------------------------------------------
  if (millis() - lastPrint >= PRINT_PERIOD_MS) {
    lastPrint = millis();

    // Long axis (body Z) expressed in the launch frame = 3rd column of R(q).
    float vx = 2.0f * (q1*q3 + q0*q2);
    float vy = 2.0f * (q2*q3 - q0*q1);
    float vz = 1.0f - 2.0f * (q1*q1 + q2*q2);   // = cos(total tilt)

    // Two orthogonal off-vertical lean angles (each 0 at vertical).
    float pitch = PITCH_SIGN * atan2f(vx, vz) * RAD2DEG;   // lean in X-Z plane
    float roll  = ROLL_SIGN  * atan2f(vy, vz) * RAD2DEG;   // lean in Y-Z plane

    // CSV: millis, pitch_deg, roll_deg, tilt_deg
    Serial.print(pitch, 2);
    Serial.print(',');
    Serial.println(roll, 2);
  }
}
