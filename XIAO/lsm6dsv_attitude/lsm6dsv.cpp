/*
 * lsm6dsv.cpp - implementation
 * Register encodings/sensitivities from ST's lsm6dsv-pid driver.
 */
#include "lsm6dsv.h"

// ---- Register addresses ----------------------------------------------------
#define REG_WHO_AM_I    0x0F
#define REG_CTRL2       0x11   // gyro ODR (ODR_G) + op mode
#define REG_CTRL3       0x12   // BDU, IF_INC
#define REG_CTRL6       0x15   // gyro full-scale (FS_G)
#define REG_STATUS      0x1E   // GDA = bit1
#define REG_OUTX_L_G    0x22

#define WHO_AM_I_VAL    0x70
#define STATUS_GDA      0x02

// ---- Configuration: change these two lines to reconfigure ------------------
// CTRL2 ODR_G (high-performance): 0x06=120, 0x07=240, 0x08=480, 0x09=960 Hz...
#define GYRO_ODR_CODE   0x07   // 240 Hz
// CTRL6 FS_G: 0x0=125, 0x1=250, 0x2=500, 0x3=1000, 0x4=2000, 0xC=4000 dps
#define GYRO_FS_CODE    0x04   // +/-2000 dps  (set 0x02 for +/-500 dps)

// FS code -> sensitivity (dps/LSB). Keeps range and scaling from desyncing.
static float fsToSensitivity(uint8_t fs) {
  switch (fs & 0x0F) {
    case 0x0: return 0.004375f;  // +/-125
    case 0x1: return 0.008750f;  // +/-250
    case 0x2: return 0.017500f;  // +/-500
    case 0x3: return 0.035000f;  // +/-1000
    case 0x4: return 0.070000f;  // +/-2000
    case 0xC: return 0.140000f;  // +/-4000
    default:  return 0.070000f;
  }
}

// ---------------------------------------------------------------------------
LSM6DSVClass::LSM6DSVClass(TwoWire &wire, uint8_t address)
  : _wire(&wire), _address(address), _gyroSens(0.070f) {}

int LSM6DSVClass::begin() {
  // NOTE: the sketch must call Wire.begin() (and setClock) once before this,
  // so multiple drivers can share the bus without re-initializing it.

  // Software reset: clears any confused digital state left by an interrupted
  // transfer (the software analog of a power cycle). Self-clears when done.
  writeReg(REG_CTRL3, 0x01);               // SW_RESET = bit0
  unsigned long t0 = millis();
  while (readReg(REG_CTRL3) & 0x01) {
    if (millis() - t0 > 10) return 0;      // reset never cleared -> let caller retry
  }

  if (readReg(REG_WHO_AM_I) != WHO_AM_I_VAL) {
    return 0;                              // wrong/missing part
  }

  _gyroSens = fsToSensitivity(GYRO_FS_CODE);

  writeReg(REG_CTRL3, 0x44);               // BDU + IF_INC (auto-increment)
  writeReg(REG_CTRL6, GYRO_FS_CODE);       // full scale
  writeReg(REG_CTRL2, GYRO_ODR_CODE);      // ODR + high-performance mode
  delay(100);                              // gyro turn-on settle
  return 1;
}

void LSM6DSVClass::end() {
  writeReg(REG_CTRL2, 0x00);               // ODR_G = 0 -> gyro power-down
}

int LSM6DSVClass::gyroscopeAvailable() {
  return (readReg(REG_STATUS) & STATUS_GDA) ? 1 : 0;
}

int LSM6DSVClass::readGyroscope(float &x, float &y, float &z) {
  uint8_t b[6];
  readRegs(REG_OUTX_L_G, b, 6);            // reading high bytes clears GDA

  int16_t rx = (int16_t)(b[1] << 8 | b[0]);
  int16_t ry = (int16_t)(b[3] << 8 | b[2]);
  int16_t rz = (int16_t)(b[5] << 8 | b[4]);

  x = rx * _gyroSens;                       // degrees per second
  y = ry * _gyroSens;
  z = rz * _gyroSens;
  return 1;
}

float LSM6DSVClass::gyroscopeSensitivity() {
  return _gyroSens;
}

// ---- low-level I2C ---------------------------------------------------------
void LSM6DSVClass::writeReg(uint8_t reg, uint8_t val) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(val);
  _wire->endTransmission();
}

uint8_t LSM6DSVClass::readReg(uint8_t reg) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->endTransmission(false);           // repeated start
  _wire->requestFrom(_address, (uint8_t)1);
  return _wire->available() ? _wire->read() : 0;
}

void LSM6DSVClass::readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->endTransmission(false);
  _wire->requestFrom(_address, len);
  for (uint8_t i = 0; i < len && _wire->available(); i++) {
    buf[i] = _wire->read();
  }
}

// Ready-to-use instance, matching `IMU` usage in the sketch.
LSM6DSVClass IMU;
