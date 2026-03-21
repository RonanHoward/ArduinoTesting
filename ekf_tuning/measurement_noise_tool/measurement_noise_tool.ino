#include "Arduino_BMI270_BMM150.h"

const unsigned long samples = 1000;


void setup() {

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    while (1);
  }

  delay(3000);

}

float ax, ay, az;
unsigned long sample_count = 0;

void loop() {
  
  if (sample_count < samples) {
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      Serial.print(ax,6);
      Serial.print(',');
      Serial.print(ay,6);
      Serial.print(',');
      Serial.println(az,6);
      sample_count++;
    }
  }

}
