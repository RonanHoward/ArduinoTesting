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
  
  if ( x*x + y*y == VECTORING_LIMIT_DEG_SQ ) {

    const float command_angle = atan2(x, y);
    x = round();
    y = round();
    Xservo.write(90 - round( VECTORING_LIMIT_DEG * cos(command_angle) * Xgain ));
    Yservo.write(90 + round( VECTORING_LIMIT_DEG * sin(command_angle) * Ygain ));

    return;
    
  }

  Xservo.write(90 - round(x*Xgain));
  Yservo.write(90 + round(y*Ygain));
}
