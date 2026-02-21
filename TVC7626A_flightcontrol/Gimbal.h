#ifndef Gimbal_h
#define Gimbal_h

#include <Servo.h>
#include <Arduino.h>

#define VECTORING_LIMIT_DEG 6.0f

class Gimbal {
  public:
    Servo Xservo;
    Servo Yservo;

    float Xgain, Ygain;
    
    Gimbal();
    Gimbal(float xgain, float ygain);

    void attach();
    
    void write(int x, int y);
};

#endif