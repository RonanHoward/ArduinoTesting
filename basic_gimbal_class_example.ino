#include <Servo.h>

class Gimbal {
  public:
    Servo Xaxis_servo;
    Servo Yaxis_servo;

    int HARD_SERVO_LIMIT;

    Gimbal() {
      HARD_SERVO_LIMIT = 30;
    }
    
    void attach() {

      Xaxis_servo.attach(D4);
      Yaxis_servo.attach(D5);

    }

    // inputs are identical to two servos as if they were oriented with the arduino
    void write(long pitch_command, long yaw_command) {

      // create temp variables to operate on and center the range (i.e. from [0,180] to [-90,90])
      float temp_pitch = pitch_command - 90;
      float temp_yaw = yaw_command - 90;

      // rotate command point 45 degrees
      long q1_servo_command = round(temp_pitch * 0.707106781187 - temp_yaw * 0.707106781187);
      long q2_servo_command = round(temp_yaw * 0.707106781187 + temp_pitch * 0.707106781187);

      // apply hard servo limit
      if (q1_servo_command > HARD_SERVO_LIMIT) {
        q1_servo_command = HARD_SERVO_LIMIT;
      } else if (q1_servo_command < -HARD_SERVO_LIMIT) {
        q1_servo_command = -HARD_SERVO_LIMIT;
      }
      if (q2_servo_command > HARD_SERVO_LIMIT) {
        q2_servo_command = HARD_SERVO_LIMIT;
      } else if (q2_servo_command < -HARD_SERVO_LIMIT) {
        q2_servo_command = -HARD_SERVO_LIMIT;
      }

      // shift back to range [0,180]
      q1_servo_command = q1_servo_command + 90;
      q2_servo_command = q2_servo_command + 90;

      // write computed commands
      Xaxis_servo.write(q1_servo_command);
      Yaxis_servo.write(q2_servo_command);

    }
  
};



Gimbal myGimbal;

void setup() {

  myGimbal.attach();
  myGimbal.write(90, 90);

}

void loop() {

  // ...

}
