#include <Servo.h>

Servo x_servo;
Servo y_servo;

void setup() {
  // put your setup code here, to run once:
  x_servo.attach(D4);
  y_servo.attach(D5);
}

int servo_radius = 20;

int x_count = -servo_radius;
int y_count = 0;

bool x_up = true;
bool x_pause = false;
bool y_down = false;
bool y_return = false;

long x_command;
long y_command;

void loop() {
  
  x_command = 90 + x_count;
  y_command = 90 + y_count;
  y_servo.write(y_command);
  x_servo.write(x_command);

  if ( !x_pause ) {
    if ( x_up ) {
      x_count = x_count + 1;
      if ( x_count > servo_radius ) x_up = false;
    } else {
      x_count = x_count - 1;
      if ( x_count < -servo_radius ) x_up = true;
    }
    x_pause = true;
    delay(10);
  }

  if ( y_return ) {
    y_count = y_count + 1;
    if ( y_count > 0 ) {
      // continue to next x value and reset cycle
      x_up = true;
      y_down = false;
      y_return = false;
      x_pause = false;
    }
  } else if ( y_down ) {
    y_count = y_count - 1;
    if ( y_count < -servo_radius ) y_return = true;
  } else {
    y_count = y_count + 1;
    if ( y_count > servo_radius ) y_down = true;
  }

  delay(10);
}
