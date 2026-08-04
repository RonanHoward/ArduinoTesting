/*
 * lps22hb.cpp - implementation
 * Register map / conversions from ST's LPS22HB definitions.
 */
#include "lps22hb.h"

// ---- Register addresses ----------------------------------------------------
#define REG_WHO_AM_I     0x0F
#define REG_CTRL_REG1    0x10   // ODR, BDU
#define REG_CTRL_REG2    0x11   // IF_ADD_INC (auto-increment), ONE_SHOT
#define REG_STATUS       0x27   // P_DA = bit0, T_DA = bit1
#define REG_PRESS_OUT_XL 0x28   // 24-bit pressure (XL/L/H), 2's complement
#define REG_TEMP_OUT_L   0x2B   // 16-bit temperature (L/H), 2's complement

#define WHO_AM_I_VAL     0xB1
#define STATUS_P_DA      0x01

// CTRL_REG1: ODR[6:4] (0x30 = 25 Hz continuous) | BDU (bit1, 0x02)
// BDU keeps the 3 pressure bytes coherent until PRESS_OUT_H is read.
#define CTRL1_VAL        0x32

// LSB scaling (from ST): pressure /4096 -> hPa, temperature /100 -> degC
static const float PRESS_LSB_PER_HPA = 4096.0f;
static const float TEMP_LSB_PER_DEGC = 100.0f;

// ---------------------------------------------------------------------------
LPS22HBClass::LPS22HBClass(TwoWire &wire, uint8_t address)
  : _wire(&wire), _address(address) {}

int LPS22HBClass::begin() {
  // Bus must already be initialized by the sketch (shared with the gyro).

  // Software reset: clears confused digital state from an interrupted
  // transfer. SWRESET self-clears when the reset completes.
  writeReg(REG_CTRL_REG2, 0x04);            // SWRESET = bit2
  unsigned long t0 = millis();
  while (readReg(REG_CTRL_REG2) & 0x04) {
    if (millis() - t0 > 10) return 0;       // never cleared -> let caller retry
  }

  if (readReg(REG_WHO_AM_I) != WHO_AM_I_VAL) {
    return 0;
  }
  // CTRL_REG2 reset value already has IF_ADD_INC = 1, so multi-byte
  // burst reads work; we just set the data rate + BDU here.
  writeReg(REG_CTRL_REG1, CTRL1_VAL);       // 25 Hz continuous, BDU on
  return 1;
}

void LPS22HBClass::end() {
  writeReg(REG_CTRL_REG1, 0x00);            // ODR = one-shot / powered down
}

int LPS22HBClass::pressureAvailable() {
  return (readReg(REG_STATUS) & STATUS_P_DA) ? 1 : 0;
}

int LPS22HBClass::readPressure(float &hPa) {
  uint8_t b[3];
  readRegs(REG_PRESS_OUT_XL, b, 3);         // XL, L, H (reading H unlocks BDU)

  int32_t raw = ((int32_t)b[2] << 16) | ((int32_t)b[1] << 8) | b[0];
  if (raw & 0x00800000) raw |= 0xFF000000;  // sign-extend 24-bit -> 32-bit
  hPa = raw / PRESS_LSB_PER_HPA;
  return 1;
}

int LPS22HBClass::readTemperature(float &degC) {
  uint8_t b[2];
  readRegs(REG_TEMP_OUT_L, b, 2);
  int16_t raw = (int16_t)(b[1] << 8 | b[0]);
  degC = raw / TEMP_LSB_PER_DEGC;
  return 1;
}

// ---- low-level I2C ---------------------------------------------------------
void LPS22HBClass::writeReg(uint8_t reg, uint8_t val) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(val);
  _wire->endTransmission();
}

uint8_t LPS22HBClass::readReg(uint8_t reg) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->endTransmission(false);
  _wire->requestFrom(_address, (uint8_t)1);
  return _wire->available() ? _wire->read() : 0;
}

void LPS22HBClass::readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->endTransmission(false);
  _wire->requestFrom(_address, len);
  for (uint8_t i = 0; i < len && _wire->available(); i++) {
    buf[i] = _wire->read();
  }
}

// Ready-to-use instance.
LPS22HBClass BARO;
