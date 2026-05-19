#ifndef Gimbal_h
#define Gimbal_h

#include <Servo.h>
#include <Arduino.h>

#define VECTORING_LIMIT_DEG 10.0f


class Gimbal {
  public:
    Servo Xservo;
    Servo Yservo;

    float Xgain, Ygain;
    
    Gimbal();
    // x = phi, y = theta
    Gimbal(float xgain, float ygain);

    void attach();
    
    // x = phi, y = theta
    void write(int x, int y);
};

#endif
