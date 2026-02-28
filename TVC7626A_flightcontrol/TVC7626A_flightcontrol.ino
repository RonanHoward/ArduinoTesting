#include "Arduino_BMI270_BMM150.h"
#include "EKF.h"
#include "Gimbal.h"
#include "PID.h"
#include "Telemetry.h"

#define XGAIN 2.1645f
#define YGAIN 3.413f

#define LASER true

#define PID_FREQUENCY_HZ 100ul
#define SERVO_UPDATE_FREQUENCY_HZ 100.0f


#define PID_PERIOD_MS (1000ul / PID_FREQUENCY_HZ)
#define SERVO_UPDATE_PERIOD_MS (1000ul / SERVO_UPDATE_FREQUENCY_HZ)


Gimbal gimbal(XGAIN, YGAIN);


// handle simultaneous events
unsigned long startTime = 0;

unsigned long last_PID_update_time = 0;


// EKF variables
EKF state;
float EKF_PREDICT_PERIOD_S;
float ax, ay, az, gx, gy, gz; // allocate for reading IMU

// PID
PID pid_X;
PID pid_Y;

// Servo
unsigned long last_servo_update = 0;


// Telemetry
Telemetry telemetry;
unsigned long last_telemetry_log = 0;
unsigned int print_log_counter = 0;



void setup() {

  #if LASER
    pinMode(D3, OUTPUT);
    digitalWrite(D3, HIGH);
  #endif

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    while (1);
  }

  gimbal.attach();

  EKF_PREDICT_PERIOD_S = 1.0f / IMU.gyroscopeSampleRate(); // in seconds

}




void loop() {

  startTime = millis();

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


  if (startTime - last_PID_update_time >= PID_PERIOD_MS) {
    pid_X.update(state.phi_rad);
    pid_Y.update(state.theta_rad);
    last_PID_update_time = startTime;
    // Serial.print(state.phi_rad);
    // Serial.print('\t');
    // Serial.print(state.theta_rad);
    // Serial.print('\t');
    // Serial.print(pid_X.get_un());
    // Serial.print('\t');
    // Serial.println(pid_Y.get_un());
  }

  if (startTime - last_servo_update >= SERVO_UPDATE_PERIOD_MS) {
    gimbal.write((int)(round(pid_X.get_un()*RAD_TO_DEG)), (int)(round(pid_Y.get_un()*RAD_TO_DEG)));
    last_servo_update = startTime;
  }


  // log telemetry
  if (startTime >= TELEMETRY_LOG_DELAY_MS) {
    if (startTime >= TELEMETRY_LOG_DELAY_MS + TELEMETRY_LOG_TIME_SEC*1000 + 100) {
      Serial.print(telemetry.logs[print_log_counter][0]);
      Serial.print('\t');
      Serial.print(telemetry.logs[print_log_counter][1]);
      Serial.print('\t');
      Serial.print(telemetry.logs[print_log_counter][2]);
      Serial.print('\t');
      Serial.print(telemetry.logs[print_log_counter][3]);
      Serial.print('\t');
      Serial.println(telemetry.logs[print_log_counter][4]);
      print_log_counter++;
      if (print_log_counter >= TELEMETRY_SAMPLES_PER_SEC*TELEMETRY_LOG_TIME_SEC) {
        print_log_counter = 0;
      }
    } else {
      if ( startTime - last_telemetry_log >= 1000/TELEMETRY_SAMPLES_PER_SEC ) {
        telemetry.log(millis(), state.phi_rad, state.theta_rad, pid_X.get_un(), pid_Y.get_un());
        last_telemetry_log = startTime;
      }
    }
  }


}
