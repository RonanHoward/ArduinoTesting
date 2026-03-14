#include "Arduino_BMI270_BMM150.h"
#include "EKF.h"
#include <avr/dtostrf.h>

const unsigned long PRINT_FREQUENCY_HZ = 10ul;

float EKF_PREDICT_PERIOD_S;
float ax, ay, az, gx, gy, gz;

unsigned long last_print = 0;

char strBuf[50];
char bufferT[20];
char bufferP[20];
EKF state;

void setup() {
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    while (1);
  }
  EKF_PREDICT_PERIOD_S = 1.0f / IMU.gyroscopeSampleRate(); // in seconds
}

void loop() {

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    // apply coordinate changes while passing into predict method
    state.lpf_gyro(gx, gy, gz);
    state.predict(EKF_PREDICT_PERIOD_S);
  }
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    // apply coordinate changes while passing into update method
    state.lpf_acc(ax, ay, -az);
    state.update();
  }

  if (micros() - last_print >= 1000ul / PRINT_FREQUENCY_HZ) {
    dtostrf(state.theta_rad*RAD_TO_DEG, 6, 2, bufferT);
    dtostrf(state.phi_rad*RAD_TO_DEG, 6, 2, bufferP);
    sprintf(strBuf, "%s,%s", bufferT, bufferP);
    Serial.println(strBuf);
  }

}
