#include "Arduino_BMI270_BMM150.h"
#include <avr/dtostrf.h>

// Filtering constants
#define GYRO_IIR_ALPHA 0.01f
// Gyro bias
#define GX_BIAS 0.001f
#define GY_BIAS 0.001f

const unsigned long PRINT_FREQUENCY_HZ = 13ul;

unsigned long last_print = 0;

float phi_deg = 0.0f;
float theta_deg = 0.0f;
float gx, gy, gz;
float pgx = 0.0f;
float pgy = 0.0f;

unsigned long last_reading;

char strBuf[50];
char bufferT[20];
char bufferP[20];

void setup() {
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    while (1);
  }
  delay(3000);
  last_reading = millis();
}

void loop() {

  if (IMU.gyroscopeAvailable()) {

    // Time when readings become available (microseconds)
    unsigned long now = micros();
    // Time since last readings (seconds)
    float T_sec = (now - last_reading) * 0.000001f;

    // Get readings
    IMU.readGyroscope(gx, gy, gz);

    // Apply filter
    gx = GYRO_IIR_ALPHA * (gx - pgx) + pgx;
    gy = GYRO_IIR_ALPHA * (gy - pgy) + pgy;

    // Estimate new current orientation
    phi_deg   += (pgx + gx) * 0.5f * T_sec;
    theta_deg += (pgy + gy) * 0.5f * T_sec;

  }

  if (millis() - last_print >= 1000ul / PRINT_FREQUENCY_HZ) {
    dtostrf(theta_deg, 6, 2, bufferT);
    dtostrf(phi_deg, 6, 2, bufferP);
    sprintf(strBuf, "%s,%s", bufferT, bufferP);
    Serial.println(strBuf);
    last_print = millis();
  }

}
