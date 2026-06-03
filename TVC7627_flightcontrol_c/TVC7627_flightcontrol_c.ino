#include <Arduino_LPS22HB.h>
#include "Arduino_BMI270_BMM150.h"
#include "Gimbal.h"
#include "PID.h"

#define SPEAKER_PIN D6
#define SPEAKER_TONE_HZ 2730


// Gyro bias
#define GX_BIAS -0.1062f
#define GY_BIAS -0.0075f
#define GZ_BIAS -0.1001f

// Gimbal
// Angle to servo angle ratios
#define XGAIN 2.1645f
#define YGAIN 3.413f

// ---- TELEMETRY ----
// phi, theta, pid_phi, pid_theta, gx, gy, barometer

// Runtime; after this many seconds, stop collecting telemetry
#define RUNTIME 60
// Telemetry same rate (Hz)
#define TELEMETRY_FREQ_HZ 10

// Number of measurements being tracked over time (not including time)
// IMPORTANT: source code must change if this is changed
#define MEASUREMENTS 5

#define MAX_TELEMETRY_ENTRIES RUNTIME*TELEMETRY_FREQ_HZ

// Ignition
#define IGNITION_PIN A5
#define THRESHOLD_VOLTAGE 1.5f // Threshold for launch (adjust this)
#define ADC_MAX 4095
#define VREF 3.3f

// Telemetry logging
unsigned long timestamps[MAX_TELEMETRY_ENTRIES];
float entries[MAX_TELEMETRY_ENTRIES][MEASUREMENTS];
unsigned long last_log;
int current_entry = 0;
unsigned long launch_time; // in microseconds from power on


// Rocket State
enum class FlightState : uint8_t {
  PRE_LAUNCH,
  FLIGHT,
  OUTPUT_TELEMETRY
};

FlightState flightState = FlightState::PRE_LAUNCH;

Gimbal gimbal(XGAIN, YGAIN);


// Create PIDs
PID pid_phi;
PID pid_theta;


// Store readings and times
float gx, gy, gz;
float pgx = 0.0f;
float pgy = 0.0f;
unsigned long last_reading;
unsigned long done_time;


// Rocket orientation state
float phi_deg = 0.0f;
float theta_deg = 0.0f;
// Attitude quaternion (body -> launch frame), [w, x, y, z]. Identity = vertical.
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;




void setup() {

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    while (1);
  }
  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    while (1);
  }

  analogReadResolution(12);

  gimbal.attach();
  
  tone(SPEAKER_PIN, SPEAKER_TONE_HZ, 500);
  delay(100);
  gimbal.write(0,0);

}

void loop() {

  if (flightState == FlightState::FLIGHT) {
    controlFlight();
  } else if (flightState == FlightState::PRE_LAUNCH) {
    detectLaunch();
  } else {
    outputTelemetry();
  }

}




void detectLaunch() {
  // Read analog pin
  // NOTE: this is not debounced, consider this in future revisions
  int raw = analogRead(IGNITION_PIN);
  float voltage = raw * (VREF / ADC_MAX);

  if (voltage >= THRESHOLD_VOLTAGE) {
    flightState = FlightState::FLIGHT;
    launch_time = micros();
    last_log = millis();
    tone(SPEAKER_PIN, SPEAKER_TONE_HZ, 500);
    last_reading = launch_time;
    done_time = launch_time + RUNTIME * 1000000;
  }
}





void controlFlight() {
  
  if (IMU.gyroscopeAvailable()) {

    IMU.readGyroscope(gx, gy, gz);

    // Real elapsed time for the step (more accurate than a fixed dt)
    unsigned long now = micros();
    float dt = (now - last_reading) * 1.0e-6f;
    last_reading = now;

    // Skip the first sample and any abnormally large step
    if (dt > 0.0f && dt < 0.1f) {

      // Remove bias and scale error
      gx = gx - GX_BIAS;
      gy = gy - GY_BIAS;
      gz = 0.0f;

      // Board axes -> body axes
      float wx = gx * DEG_TO_RAD;  // rad/s, body X (transverse)
      float wy = gy * DEG_TO_RAD;  // rad/s, body Y (transverse)
      float wz = 0.0f;  // rad/s, body Z (long axis / spin)

      // Integrate with the exact exponential map. A body rate that is constant
      // over the step (e.g. steady spin) is integrated exactly at any dt;
      // residual error comes only from the rate changing within a step.
      float dthx = wx * dt;
      float dthy = wy * dt;
      float dthz = 0.0f;
      float theta = sqrtf(dthx*dthx + dthy*dthy);

      float dw, dx, dy, dz;           // incremental rotation quaternion
      if (theta > 1.0e-8f) {
        float half = 0.5f * theta;
        float s = sinf(half) / theta;
        dw = cosf(half);
        dx = dthx * s;
        dy = dthy * s;
        dz = 0.0f;
      } else {                        // small-angle limit (avoids div-by-zero)
        dw = 1.0f;
        dx = 0.5f * dthx;
        dy = 0.5f * dthy;
        dz = 0.0f;
      }

      // q = q (x) dq   (Hamilton product; body-frame rate update)
      float nq0 = q0*dw - q1*dx - q2*dy - q3*dz;
      float nq1 = q0*dx + q1*dw + q2*dz - q3*dy;
      float nq2 = q0*dy - q1*dz + q2*dw + q3*dx;
      float nq3 = q0*dz + q1*dy - q2*dx + q3*dw;

      // Renormalise
      float inv = 1.0f / sqrtf(nq0*nq0 + nq1*nq1 + nq2*nq2 + nq3*nq3);
      q0 = nq0*inv; q1 = nq1*inv; q2 = nq2*inv; q3 = nq3*inv;
    }

    // Long axis (body Z) expressed in the launch frame = 3rd column of R(q).
    float vx = 2.0f * (q1*q3 + q0*q2);
    float vy = 2.0f * (q2*q3 - q0*q1);
    float vz = 1.0f - 2.0f * (q1*q1 + q2*q2);   // = cos(total tilt)

    // Two orthogonal off-vertical lean angles (each 0 at vertical).
    float pitch = atan2f(vx, vz);   // lean in X-Z plane
    float roll  = atan2f(vy, vz);   // lean in Y-Z plane

    // Update PIDs
    pid_phi.update(roll);
    pid_theta.update(pitch);

    // Write PID output to gimbal
    gimbal.write(-round(pid_phi.get_un()*RAD_TO_DEG), round(pid_theta.get_un()*RAD_TO_DEG));

    // Is runtime over?
    if (now >= done_time) {
      // Take a break
      delay(20000); // 20s
      flightState = FlightState::OUTPUT_TELEMETRY;
    }

    // Collect Telemetry
    if (millis() - last_log >= 1000/TELEMETRY_FREQ_HZ) {
      if (current_entry < MAX_TELEMETRY_ENTRIES) {
        float pressure = BARO.readPressure();
        timestamps[current_entry] = now;
        entries[current_entry][0] = pitch;
        entries[current_entry][1] = roll;
        entries[current_entry][2] = pid_phi.get_un();
        entries[current_entry][3] = pid_theta.get_un();
        entries[current_entry][4] = pressure;
        current_entry++;
      }
      last_log = millis();
    }

    pgx = gx;
    pgy = gy;
    last_reading = now;

  }

}



void outputTelemetry() {
  Serial.println();
  Serial.print("Launch time:");
  Serial.print('\t');
  Serial.println(launch_time);
  for (int i = 0; i < MAX_TELEMETRY_ENTRIES; i++) {
    Serial.print((timestamps[i] - launch_time) * 0.000001f, 6); // ms -> s
    Serial.print(',');
    for (int j = 0; j < MEASUREMENTS-1; j++) {
      Serial.print(entries[i][j],6);
      Serial.print(',');
    }
    Serial.println(entries[i][MEASUREMENTS-1],6);
  }
  delay(10000); // 10s then print again
}

