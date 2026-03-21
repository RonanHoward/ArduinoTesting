#include "Arduino_BMI270_BMM150.h"
#include "Gimbal.h"

#define LAUNCH_THRESHOLD 0.96f // in g's (1g=9.8m/s^2), reading will be roughly 1g during rest

#define ALPHA_ACC_IIR 0.8f // alpha for accelerometer IIR

#define XGAIN 2.1645f
#define YGAIN 3.413f

#define g 9.81f

bool launched = false;

Gimbal gimbal(XGAIN, YGAIN);

float ax, ay, az, gx, gy, gz; // store raw sensor readings
// previous accelerations for IIR
float pax = 0;
float pay = 0;
float paz = 1;
// previous previous acceleration to compute initial state (before filter lag)
float ppax, ppay, ppaz;

float phi_deg;
float theta_deg;

unsigned long now;
unsigned long last_gyro_reading;
long T = 100;

void setup() {
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    while (1);
  }

  gimbal.attach();

}

void loop() {

  if (launched) {

    now = millis();

    if (IMU.gyroscopeAvailable()) {

      IMU.readGyroscope(gx, gy, gz);

      T = (now - last_gyro_reading) / 1000; // time between readings
      
      // NOTE: (up late making sure of this)
      // +gx = +phi
      // +gy = +theta

      Serial.print(gx);
      Serial.print(',');
      Serial.print(gy);
      Serial.print(',');
      Serial.println(gz);

      // TODO: Gyro state estimation

      last_gyro_reading = now;
    }
    
  } else {

    // 100Hz (slightly lower in reality)
    if (IMU.accelerationAvailable()){

      IMU.readAcceleration(ax, ay, az);
      
      // First apply IIR filter to az and immediately check if we have launched
      az = ALPHA_ACC_IIR * paz + (1-ALPHA_ACC_IIR) * az;
      if (az < LAUNCH_THRESHOLD) {
        launched = true;
        // set initial orientation
        phi_deg   = -atan(ppay/ppaz) * RAD_TO_DEG;
        theta_deg = asin(ppax) * RAD_TO_DEG;
        return;
      }

      // Apply IIR to rest and save previous accelerations
      ax = ALPHA_ACC_IIR * pax + (1-ALPHA_ACC_IIR) * ax;
      ay = ALPHA_ACC_IIR * pay + (1-ALPHA_ACC_IIR) * ay;
      ppax = pax;
      ppay = pay;
      ppaz = paz;
      pax = ax;
      pay = ay;
      paz = az;
    
    }

  }


}
