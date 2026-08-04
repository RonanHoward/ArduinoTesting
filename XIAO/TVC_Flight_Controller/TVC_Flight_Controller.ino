/*
 * TVC Rocket Flight Controller  -  top level
 *
 * This file is intentionally tiny: it just starts the flight controller
 * and runs it. All settings are in config.h; all behavior is in the
 * modules (FlightController.h and friends). See README.md.
 *
 * Hardware: XIAO RA4M1 + LSM6DSV gyro + LPS22HB baro + 3 servos.
 */
#include "FlightController.h"

FlightController flightController;

void setup() {
  Serial.begin(115200);
  // Don't wait forever for a USB host - the rocket flies untethered.
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) { /* brief wait for bench use */ }

  flightController.begin();
}

void loop() {
  flightController.update();
}
