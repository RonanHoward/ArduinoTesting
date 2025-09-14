#include "EKF_RollPitch.h"
#include "Gimbal_PID.h"
#include "Gimbal.h"
#include <Arduino_BMI270_BMM150.h>

// Combo editorials
#define SERVO_WRITE_GAIN 0.7

// EKF setup
#define PREDICT_PERIOD_MS 10
#define UPDATE_PERIOD_MS 50
#define PRINT_PERIOD_MS 50

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

// PID setup
Gimbal_PID pid_x;
Gimbal_PID pid_y;
unsigned long pid_last_time = 0;
const float pid_frequency = 1.0f / 0.001f;

// Gimbal setup
Gimbal gimbal;

void setup() {
  Serial.begin(9600);

  delay(1000);

  // Startup calibration
  Serial.println("Calibrating...");

  pinMode(D3, OUTPUT);
  digitalWrite(D3, HIGH);
  
  gimbal.attach();

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    while (1);
  }

  predictTimer = millis();
  updateTimer  = millis();
  pid_last_time = micros();
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

  // update PID
  if (micros() - pid_last_time >= pid_frequency) {
    pid_last_time = micros();
    pid_x.update(ahrs.theta_rad);
    pid_y.update(ahrs.phi_rad);
  }


  // write/print
  if (millis() - printTimer >= PRINT_PERIOD_MS) {
    gimbal.write(-pid_y.get_un()*SERVO_WRITE_GAIN, -pid_x.get_un()*SERVO_WRITE_GAIN);
    printTimer += PRINT_PERIOD_MS;
  }
}
