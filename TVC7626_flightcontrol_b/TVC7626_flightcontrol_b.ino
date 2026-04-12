#include "Arduino_BMI270_BMM150.h"
#include "Gimbal.h"
#include "PID.h"

// First-order fading memory IIR filter applied to gyroscope
#define GYRO_IIR_ALPHA 0.01f

// Launch detection sensitivity
#define LAUNCH_THRESHOLD 1.1f // should be slightly above 1g

// Gimbal angle to servo angle ratios
#define XGAIN 2.1645f
#define YGAIN 3.413f

// Gyro bias
#define GX_BIAS 0.001f
#define GY_BIAS 0.001f

// ---- TELEMETRY ----
// phi, theta, pid_phi, pid_theta, gx, gy, barometer

// Runtime; after this many seconds, stop collecting telemetry
#define RUNTIME 40
// Telemetry same rate (Hz)
#define TELEMETRY_FREQ_HZ 10

// Number of measurements being tracked over time (not including time)
// IMPORTANT: source code must change if this is changed
#define MEASUREMENTS 7
// Number of measurements to be logged once with no time stamp
#define ONE_SHOT_ENTRIES 2

#define MAX_TELEMETRY_ENTRIES RUNTIME*TELEMETRY_FREQ_HZ



// ---- Code ----

Gimbal gimbal(XGAIN, YGAIN);

PID pid_phi;
PID pid_theta;

bool launched = false;

// Gyroscope readings (gx, gy, gz)
float gx, gy, gz; // +gx = +phi, +gy = +theta
// Previous gyroscope readings (pgx, pgy)
float pgx, pgy;
// Accelerometer readings vector (ax, ay, az)
float ax, ay, az;
// Previous accelerometer readings vector (pax, pay, paz)
float pax, pay, paz;

// Current euler orientation
float phi_deg, theta_deg;

// Last readings (microseconds)
unsigned long last_reading;
// Time when algorithm stops, computed when launch time is found
unsigned long done_time;

// Telemetry logging
unsigned long timestamps[MAX_TELEMETRY_ENTRIES];
float entries[MAX_TELEMETRY_ENTRIES][MEASUREMENTS];
int current_entry = 0;
unsigned long launch_time; // in microseconds from power on
float launch_phi_deg, launch_theta_deg;


void setup() {

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    while (1);
  }

  gimbal.attach();

  delay(100);
  gimbal.write(0,0);
  delay(1000);
  last_reading = micros();
}

void loop() {

  if (launched) {

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

      // Update PIDs
      pid_phi.update(phi_deg * DEG_TO_RAD);
      pid_theta.update(theta_deg * DEG_TO_RAD);

      // Write PID output to gimbal
      gimbal.write(round(pid_phi.get_un()*RAD_TO_DEG), round(pid_theta.get_un()*RAD_TO_DEG));

      // Is runtime over?
      if (now >= done_time) {
        // Take a break
        delay(20000); // 20s
        // Convert units
        for (int i = 0; i < MAX_TELEMETRY_ENTRIES; i++) {
          entries[i][2] *= RAD_TO_DEG; // rad -> deg
          entries[i][3] *= RAD_TO_DEG; // rad -> deg
        }
        // Infinite loop print telemetry
        while (1) {
          Serial.println();
          Serial.print("Orientation at Launch:");
          Serial.print('\t');
          Serial.print(launch_phi_deg);
          Serial.print(',');
          Serial.println(launch_theta_deg);
          for (int i = 0; i < MAX_TELEMETRY_ENTRIES; i++) {
            Serial.print((timestamps[i] - launch_time) * 0.000001f, 6); // ms -> s
            Serial.print(',');
            for (int j = 0; j < MEASUREMENTS-1; j++) {
              Serial.print(entries[i][0],6);
              Serial.print(',');
            }
            Serial.println(entries[i][MEASUREMENTS-1]);
          }
          delay(10000);
        }
      }

      // Collect Telemetry
      if (current_entry < MAX_TELEMETRY_ENTRIES) {
        timestamps[current_entry] = now;
        entries[current_entry][0] = phi_deg;
        entries[current_entry][1] = theta_deg;
        entries[current_entry][2] = pid_phi.get_un();
        entries[current_entry][3] = pid_theta.get_un();
        entries[current_entry][4] = gx;
        entries[current_entry][5] = gy;
        entries[current_entry][6] = 0.0f;
        current_entry++;
      }

      // Save data for next time reaadings become available
      pgx = gx;
      pgy = gy;
      last_reading = now;

    }

  } else {
    // Check for new readings
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      // Time when readings become available (microseconds)
      unsigned long now = micros();
      // Time since last readings (microseconds)
      unsigned long T = now - last_reading;

      // Get readings
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);

      // Check for launch
      if (ax*ax+ay*ay+az*az > LAUNCH_THRESHOLD * LAUNCH_THRESHOLD) {

        // Time since last reading in seconds
        float T_sec = T * 0.000001f;

        // Find initial orientation from moment before launch is detected
        // Add one integration step to estimate orientation at launch detection
        phi_deg   = -atan(pay/paz) * RAD_TO_DEG + (pgx + gx) * 0.5f * T_sec;
        theta_deg = asin(pax) * RAD_TO_DEG + (pgy + gy) * 0.5 * T_sec;

        // Start TVC
        launched = true;
        launch_phi_deg   = phi_deg;
        launch_theta_deg = theta_deg;
        launch_time = now;
        done_time = now + RUNTIME * 1000000;
        last_reading = now;
        pgx = gx;
        pgy = gy;
        return;

      }

      // Save values in case we are launching on the next read
      pax = ax;
      pay = ay;
      paz = az;
      pgx = gx;
      pgy = gy;
      last_reading = now;
    }
  }

}
