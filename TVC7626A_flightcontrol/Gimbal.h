#ifndef Gimbal_h
#define Gimbal_h

#include <Servo.h>
#include <Arduino.h>

#define VECTORING_LIMIT_DEG 10.0f
#define VECTORING_LIMIT_DEG_SQ (VECTORING_LIMIT_DEG * VECTORING_LIMIT_DEG)


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