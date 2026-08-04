/*
 * Gyro-only attitude estimation + barometer on the XIAO RA4M1.
 * Two devices share one I2C bus (Wire):
 *   - LSM6DSV gyro  @ 0x6A  (lsm6dsv.h / .cpp)  serviced at 240 Hz
 *   - LPS22HB baro  @ 0x5C  (lps22hb.h / .cpp)  serviced at ~25 Hz
 * All five files live in this same sketch folder.
 *
 * The sketch initializes the shared bus ONCE; each driver's begin()
 * only configures its own chip. Distinct addresses -> no bus conflict.
 */
#include "lsm6dsv.h"
#include "lps22hb.h"
#include <math.h>

// ---- Estimator state -------------------------------------------------------
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // orientation quaternion [w,x,y,z]
float biasX = 0, biasY = 0, biasZ = 0;     // gyro bias, dps
unsigned long tPrev = 0;
static const float DEG2RAD = 0.01745329252f;

// ---- Latest barometer readings (updated at ~25 Hz) -------------------------
float lastPressure = 0.0f;   // hPa
float lastTemp     = 0.0f;   // degC

// Integrate one angular-rate sample (rad/s) over dt (s) into q (exp. map).
void integrate(float wx, float wy, float wz, float dt) {
  float wmag = sqrtf(wx*wx + wy*wy + wz*wz);
  float dq0, dq1, dq2, dq3;
  if (wmag > 1e-8f) {
    float half = 0.5f * wmag * dt;
    float s = sinf(half) / wmag;
    dq0 = cosf(half); dq1 = wx*s; dq2 = wy*s; dq3 = wz*s;
  } else {
    dq0 = 1.0f; dq1 = dq2 = dq3 = 0.0f;
  }
  float nw = q[0]*dq0 - q[1]*dq1 - q[2]*dq2 - q[3]*dq3;
  float nx = q[0]*dq1 + q[1]*dq0 + q[2]*dq3 - q[3]*dq2;
  float ny = q[0]*dq2 - q[1]*dq3 + q[2]*dq0 + q[3]*dq1;
  float nz = q[0]*dq3 + q[1]*dq2 - q[2]*dq1 + q[3]*dq0;
  float inv = 1.0f / sqrtf(nw*nw + nx*nx + ny*ny + nz*nz);
  q[0] = nw*inv; q[1] = nx*inv; q[2] = ny*inv; q[3] = nz*inv;
}

// Non-blocking periodic work. Must not block (no delay/long spins).
void otherTasks() {
  // --- Barometer service: check at ~25 Hz, not at gyro rate ---
  static unsigned long lastBaro = 0;
  if (millis() - lastBaro >= 40) {          // ~25 Hz
    lastBaro = millis();
    if (BARO.pressureAvailable()) {
      BARO.readPressure(lastPressure);      // hPa
      BARO.readTemperature(lastTemp);       // degC
    }
  }

  // --- Serial readout ~20 Hz ---
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 50) {
    lastPrint = millis();
    float roll  = atan2f(2*(q[0]*q[1] + q[2]*q[3]),
                         1 - 2*(q[1]*q[1] + q[2]*q[2]));
    float sinp  = 2*(q[0]*q[2] - q[3]*q[1]);
    float pitch = (fabsf(sinp) >= 1.0f) ? copysignf(PI/2, sinp) : asinf(sinp);
    float yaw   = atan2f(2*(q[0]*q[3] + q[1]*q[2]),
                         1 - 2*(q[2]*q[2] + q[3]*q[3]));
    Serial.print("R:");    Serial.print(roll  / DEG2RAD, 1);
    Serial.print(" P:");   Serial.print(pitch / DEG2RAD, 1);
    Serial.print(" Y:");   Serial.print(yaw   / DEG2RAD, 1);
    Serial.print("  Press:"); Serial.print(lastPressure, 2); Serial.print("hPa");
    Serial.print(" Temp:");   Serial.print(lastTemp, 1);     Serial.println("C");
  }
}

// Recover a stuck I2C bus. If a reset interrupts a read mid-byte, a slave
// can be left holding SDA low, which blocks the START of every future
// transaction (survives MCU reset; only a power cycle clears it otherwise).
// Manually clock SCL up to 9 times to let the slave finish its byte and
// release SDA, then issue a STOP. Call this BEFORE Wire.begin().
// Lines are driven open-drain style: release = INPUT_PULLUP, low = OUTPUT LOW.
void i2cBusRecovery(uint8_t sdaPin, uint8_t sclPin) {
  pinMode(sclPin, INPUT_PULLUP);
  pinMode(sdaPin, INPUT_PULLUP);
  delayMicroseconds(10);

  // Clock out up to 9 pulses while SDA is held low by a stuck slave.
  for (int i = 0; i < 9 && digitalRead(sdaPin) == LOW; i++) {
    pinMode(sclPin, OUTPUT);
    digitalWrite(sclPin, LOW);              // SCL low
    delayMicroseconds(5);
    pinMode(sclPin, INPUT_PULLUP);          // SCL released high
    delayMicroseconds(5);
  }

  // Generate a STOP condition: SDA low -> high while SCL is high.
  pinMode(sdaPin, OUTPUT);
  digitalWrite(sdaPin, LOW);
  delayMicroseconds(5);
  pinMode(sclPin, INPUT_PULLUP);            // SCL high
  delayMicroseconds(5);
  pinMode(sdaPin, INPUT_PULLUP);            // SDA released high = STOP
  delayMicroseconds(5);
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // Self-healing bring-up (no power cycle needed):
  //   1. clock a stuck slave free (frees SDA at the bus level)
  //   2. Wire.begin() reclaims the pins for I2C
  //   3. each driver's begin() software-resets its chip, then verifies it
  // Retry a few times so a lockup from an interrupted reset recovers here.
  bool ok = false;
  for (int attempt = 1; attempt <= 3 && !ok; attempt++) {
    i2cBusRecovery(SDA, SCL);               // GPIO bit-bang: free the bus
    Wire.begin();                           // (re)claim the pins for I2C
    Wire.setClock(400000);                  // Fast Mode; both parts tolerate it
    ok = IMU.begin() && BARO.begin();       // SW-reset + WHO_AM_I on each
    if (!ok) delay(10);
  }
  if (!ok) {
    Serial.println("Sensor bring-up failed after recovery. Check wiring/pull-ups.");
    while (1) delay(1000);
  }

  // Gyro bias calibration (hold still).
  Serial.println("Calibrating gyro bias -- hold the sensor still...");
  const int N = 500;
  double sx = 0, sy = 0, sz = 0;
  int c = 0;
  while (c < N) {
    if (IMU.gyroscopeAvailable()) {
      float gx, gy, gz;
      IMU.readGyroscope(gx, gy, gz);
      sx += gx; sy += gy; sz += gz; c++;
    }
  }
  biasX = sx / N; biasY = sy / N; biasZ = sz / N;

  tPrev = micros();
}

void loop() {
  // Non-blocking gyro service (fast path, 240 Hz).
  if (IMU.gyroscopeAvailable()) {
    float gx, gy, gz;
    IMU.readGyroscope(gx, gy, gz);          // dps

    unsigned long now = micros();
    float dt = (now - tPrev) * 1e-6f;
    tPrev = now;

    float wx = (gx - biasX) * DEG2RAD;
    float wy = (gy - biasY) * DEG2RAD;
    float wz = (gz - biasZ) * DEG2RAD;
    integrate(wx, wy, wz, dt);
  }

  otherTasks();                             // baro + printing, unblocked
}
