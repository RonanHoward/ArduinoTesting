/*
 * lps22hb.h - minimal I2C driver for the LPS22HB / LPS22HBTR barometer
 * API mirrors the LSM6DSV gyro driver.
 *
 * IMPORTANT: this driver does NOT call Wire.begin(). The sketch must
 * initialize the shared bus once (Wire.begin(); Wire.setClock(...)).
 * Place this file in the same folder as your .ino sketch.
 */
#ifndef LPS22HB_h
#define LPS22HB_h

#include <Arduino.h>
#include <Wire.h>

class LPS22HBClass {
  public:
    // Defaults: I2C bus = Wire, address 0x5C (SA0 = LOW; use 0x5D if SA0 high).
    LPS22HBClass(TwoWire &wire = Wire, uint8_t address = 0x5C);

    int begin();                          // 1 = ok, 0 = WHO_AM_I fail
    void end();                           // power down (back to one-shot)

    int pressureAvailable();              // 1 = new pressure sample ready
    int readPressure(float &hPa);         // hectopascals, 1 = ok
    int readTemperature(float &degC);     // degrees Celsius, 1 = ok

  private:
    TwoWire *_wire;
    uint8_t  _address;

    void    writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg(uint8_t reg);
    void    readRegs(uint8_t reg, uint8_t *buf, uint8_t len);
};

extern LPS22HBClass BARO;   // ready-to-use instance (Wire, 0x5C)

#endif
