#include "Arduino_BMI270_BMM150.h"
#include "Gimbal.h"
#include "PID.h"



// Filtering constants
#define GYRO_IIR_ALPHA 0.97f
// Gyro bias
#define GX_BIAS 0.120575f
#define GY_BIAS 0.047852f

// Gimbal
// Angle to servo angle ratios
#define XGAIN 2.1645f
#define YGAIN 3.413f

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

// Ignition
#define IGNITION_PIN A0
#define THRESHOLD_VOLTAGE 2.5f // Threshold for launch (adjust this)
#define ADC_MAX 4095
#define VREF 3.3f

// Telemetry logging
unsigned long timestamps[MAX_TELEMETRY_ENTRIES];
float entries[MAX_TELEMETRY_ENTRIES][MEASUREMENTS];
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




void setup() {

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    while (1);
  }

  analogReadResolution(12);

  gimbal.attach();

  delay(100);
  gimbal.write(0,0);
  delay(1000);

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
    last_reading = launch_time;
    done_time = launch_time + RUNTIME * 1000000;
  }
}





void controlFlight() {
  
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
      flightState = FlightState::OUTPUT_TELEMETRY;
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
      Serial.print(entries[i][0],6);
      Serial.print(',');
    }
    Serial.println(entries[i][MEASUREMENTS-1]);
  }
  delay(10000); // 10s then print again
}


