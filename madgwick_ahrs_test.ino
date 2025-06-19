#include <Arduino_BMI270_BMM150.h>
#include <MadgwickAHRS.h>

Madgwick filter;

float acceleration_sample_rate;
float gyroscope_sample_rate;
float magnetometer_sample_rate;

unsigned long microsPerReading, microsPrevious;

// float avg_sample_rate; // TODO: Kalman filter?

void setup() {

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }
  // get sample rates
  acceleration_sample_rate = IMU.accelerationSampleRate();
  gyroscope_sample_rate = IMU.gyroscopeSampleRate();
  magnetometer_sample_rate = IMU.magneticFieldSampleRate();
  // print sample rates
  Serial.print("Accelerometer sample rate = ");
  Serial.print(acceleration_sample_rate);
  Serial.println("Hz");

  Serial.print("Gyro sample rate = ");
  Serial.print(gyroscope_sample_rate);
  Serial.println("Hz");

  Serial.print("Magnetometer sample rate = ");
  Serial.print(magnetometer_sample_rate);
  Serial.println("Hz");

  // find avg sample rate (use kalman filter later)
  // avg_sample_rate = ( acceleration_sample_rate + gyroscope_sample_rate + magnetometer_sample_rate ) / 3.0;

  microsPerReading = 1000000 / acceleration_sample_rate;
  microsPrevious = micros();
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;
  unsigned long microsNow;

  microsNow = micros();

  // check if time for new calculation
  if (microsNow - microsPrevious >= microsPerReading) {
    // verify that data is ready
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);

      filter.updateIMU(gx, gy, gz, ax, ay, az);

      roll = filter.getRoll();
      pitch = filter.getPitch();
      heading = filter.getYaw();
      Serial.print("Orientation: ");
      Serial.print(heading);
      Serial.print(" ");
      Serial.print(pitch);
      Serial.print(" ");
      Serial.println(roll);

      // increment time
      microsPrevious = microsPrevious + microsPerReading;
    } else {
      Serial.println("Unable to read IMU value(s)");
    }
  }
  
}
