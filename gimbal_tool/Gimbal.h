#ifndef Gimbal_h
#define Gimbal_h

#include <Servo.h>
#include <Arduino.h>

class Gimbal {
  public:
    Gimbal();

    Servo servo_roll;
    Servo servo_pitch;

    void attach();
    void write(float vector_roll, float vector_pitch);
};

#endif
