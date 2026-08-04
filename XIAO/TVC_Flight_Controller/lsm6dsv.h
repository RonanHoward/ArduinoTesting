/*
 * lsm6dsv.h - minimal I2C driver for the LSM6DSV gyroscope
 * API modeled on the Arduino_LSM6DS3 library.
 *
 * Place this file in the same folder as your .ino sketch.
 */
#ifndef LSM6DSV_H
#define LSM6DSV_H

#include <Arduino.h>
#include <Wire.h>

class LSM6DSVClass {
public:
  // Defaults: I2C bus = Wire, address 0x6A (SA0 = LOW).
  LSM6DSVClass(TwoWire &wire = Wire, uint8_t address = 0x6A);

  int   begin();                                  // 1 = ok, 0 = WHO_AM_I fail
  void  end();                                    // power down the gyro

  int   gyroscopeAvailable();                     // 1 = new sample ready
  int   readGyroscope(float &x, float &y, float &z); // returns dps, 1 = ok

  float gyroscopeSensitivity();                   // dps per LSB (current range)

private:
  TwoWire *_wire;
  uint8_t  _address;
  float    _gyroSens;                             // dps/LSB, set in begin()

  void    writeReg(uint8_t reg, uint8_t val);
  uint8_t readReg(uint8_t reg);
  void    readRegs(uint8_t reg, uint8_t *buf, uint8_t len);
};

extern LSM6DSVClass IMU;   // ready-to-use instance (Wire, 0x6A)

#endif // LSM6DSV_H
