#ifndef Gimbal_h
#define Gimbal_h

#include <Servo.h>
#include <Arduino.h>

class Gimbal {
  public:
    Servo Xaxis_servo;
    Servo Yaxis_servo;
    int HARD_SERVO_LIMIT = 30;
    Gimbal();
    void attach();
    void write(float theta, float phi);
};

#endif
