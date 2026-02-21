#include "Arduino_BMI270_BMM150.h"
#include "EKF.h"
#include "Gimbal.h"


#define PID_FREQUENCY_HZ 100ul
#define PID_PERIOD_MS 1000ul / PID_FREQUENCY_HZ


Gimbal gimbal;


// handle simultaneous events
unsigned long startTime = 0;

unsigned long last_PID_update_time = 0;


// EKF variables
EKF state;
float EKF_PREDICT_PERIOD;
float ax, ay, az, gx, gy, gz; // allocate for reading IMU



void setup() {

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    while (1);
  }

  gimbal.attach();

  EKF_PREDICT_PERIOD = 1.0f / IMU.gyroscopeSampleRate(); // in seconds

}






void loop() {

  startTime = millis();

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    state.predict(gx, gy, gz, EKF_PREDICT_PERIOD);
  }
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
  }

  if (startTime - last_PID_update_time >= PID_PERIOD_MS) {
    // update PID
  }


}
