#ifndef Gimbal_h
#define Gimbal_h

#include <Servo.h>
#include <Arduino.h>

class Gimbal {
  public:
    Servo Xservo;
    Servo Yservo;
    
    Gimbal();

    void attach();
    
    void write(unsigned long x, unsigned long y);
};

#endif
