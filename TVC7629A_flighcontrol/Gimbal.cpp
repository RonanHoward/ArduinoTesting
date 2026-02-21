#include "Gimbal.h"
#include <Servo.h>
#include <Arduino.h>

Gimbal::Gimbal() {
  // ...
}

void Gimbal::attach() {
  Xservo.attach(D4);
  Yservo.attach(D5);
}

void Gimbal::write(unsigned long x, unsigned long y) {
  Xservo.write(x + 90);
  Yservo.write(y + 90);
}
