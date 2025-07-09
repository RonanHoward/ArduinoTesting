#include <Arduino_BMI270_BMM150.h>

const int NUMBER_OF_SAMPLES = 2000;

int count = 0;

float gx, gy, gz = 0;
// sum accumulator
double sx, sy, sz = 0;


void setup() {
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }
}

void loop() {
  if ( count < NUMBER_OF_SAMPLES && IMU.gyroscopeAvailable() ) {
    IMU.readGyroscope(gx, gy, gz);
    sx += gx;
    sy += gy;
    sz += gz;
    count++;
    Serial.println(count);
  }
  if ( count == NUMBER_OF_SAMPLES) {
    Serial.print("GYRO X OFFSET: ");
    Serial.println(sx / NUMBER_OF_SAMPLES);
    Serial.print("GYRO Y OFFSET: ");
    Serial.println(sy / NUMBER_OF_SAMPLES);
    Serial.print("GYRO Z OFFSET: ");
    Serial.println(sz / NUMBER_OF_SAMPLES);
    while (1)
      ;
  }
}
