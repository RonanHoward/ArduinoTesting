#include "EKF_RollPitch.h"
#include <Arduino_BMI270_BMM150.h>

#define PREDICT_PERIOD_MS 10
#define UPDATE_PERIOD_MS 20
#define PRINT_PERIOD_MS 250


float predict_period_sec = 0.001f * PREDICT_PERIOD_MS;
float update_period_sec = 0.001f * UPDATE_PERIOD_MS;

float P0 = 0.1f;
float Q0[2] = {0.001f, 0.001f};
float R0[3] = {0.011f, 0.011f, 0.011f};

unsigned long predictTimer;
unsigned long updateTimer;
unsigned long printTimer;

float ax, ay, az, gx, gy, gz;

EKF_RollPitch ahrs(P0, Q0, R0);

void setup() {
  Serial.begin(9600);

  delay(1000);

  // Startup calibration
  Serial.println("Calibrating...");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    while (1);
  }

  predictTimer = millis();
  updateTimer  = millis();

}

void loop() {

  // Read IMU
  if (IMU.accelerationAvailable())
    IMU.readAcceleration(ax, ay, az);
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    gx = gx * DEG_TO_RAD;
    gy = gy * DEG_TO_RAD;
    gz = gz * DEG_TO_RAD;
  }

  // Predict/update EKF filter
  if (millis() - predictTimer >= PREDICT_PERIOD_MS) {
    ahrs.predict(gx, gy, -gz, predict_period_sec);
    predictTimer += PREDICT_PERIOD_MS;
  }
  if (millis() - updateTimer >= UPDATE_PERIOD_MS) {
    ahrs.update(ax, ay, -az);
    updateTimer += UPDATE_PERIOD_MS;    
  }


  if (millis() - printTimer >= PRINT_PERIOD_MS) {
    Serial.print(ahrs.theta_rad * RAD_TO_DEG);
    Serial.print('\t');
    Serial.println(ahrs.phi_rad * RAD_TO_DEG);
    printTimer += PRINT_PERIOD_MS;
  }
}
