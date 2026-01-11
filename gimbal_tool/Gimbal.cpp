#include "Gimbal.h"
#include <Servo.h>
#include <Arduino.h>

Gimbal::Gimbal() {

}

void Gimbal::attach() {
  servo_roll.attach(D4);
  servo_pitch.attach(D5);
}

void Gimbal::write(float vector_roll, float vector_pitch) {
  servo_roll.write(round(vector_roll * 1.748)+90);
  servo_pitch.write(round(vector_pitch * 2.732)+90);
}
