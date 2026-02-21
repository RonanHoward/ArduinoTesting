#include "Gimbal.h"
#include <Servo.h>
#include <Arduino.h>

Gimbal::Gimbal() {
  Xgain = 1.0f;
  Ygain = 1.0f;
}

Gimbal::Gimbal(float xgain, float ygain) {
  Xgain = xgain;
  Ygain = ygain;
}

void Gimbal::attach() {
  Xservo.attach(D5);
  Yservo.attach(D4);
}

void Gimbal::write(int x, int y) {
  if (x > VECTORING_LIMIT_DEG) {
    x = VECTORING_LIMIT_DEG;
  } else if (x < -VECTORING_LIMIT_DEG) {
    x = -VECTORING_LIMIT_DEG;
  }
  if (y > VECTORING_LIMIT_DEG) {
    y = VECTORING_LIMIT_DEG;
  } else if (y < -VECTORING_LIMIT_DEG) {
    y = -VECTORING_LIMIT_DEG;
  }
  Xservo.write(90 - round(x*Xgain));
  Yservo.write(90 + round(y*Ygain));
}
