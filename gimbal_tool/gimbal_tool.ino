#include <Servo.h>
#include "Gimbal.h"

Gimbal gimbal;

String command;

int x, y = 0; // x-roll, y-pitch

const int SERVO_UPDATE_INTERVAL = 10; // in ms

bool playing_animation = false;
String current_animation;
unsigned long last_animation_frame = 0;
unsigned long now;
const float TIME_SCALE = 0.006f; // from ms

void setup() {

  Serial.begin(9600);

  delay(1000);

  gimbal.attach();

  pinMode(D3, OUTPUT);
  digitalWrite(D3, HIGH);

  gimbal.write(0,0);

  Serial.println("Started. Enter a command.");

}

void loop() {

  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("x")) {
      x = command.substring(2).toInt();
      gimbal.write(x, y);
      printpos();
    } else if (command.startsWith("y")) {
      y = command.substring(2).toInt();
      gimbal.write(x, y);
      printpos();
    } else if (command.equals("center")) {
      gimbal.write(0, 0);
      x = 0;
      y = 0;
      Serial.println("(0,0)");
    } else if (command.startsWith("goto ")) {
      String pos = command.substring(5);
      int di = pos.indexOf(" ");
      x = pos.substring(0,di).toFloat();
      y = pos.substring(di).toFloat();
      gimbal.write(x, y);
      printpos();
    } else if (command.startsWith("play ")) {
      String animation = command.substring(5);
      if (animation.equals("xline")) {
        current_animation = "xline";
        playing_animation = true;
      } else if (animation.equals("yline")) {
        current_animation = "yline";
        playing_animation = true;
      } else {
        Serial.print("Animation \"");
        Serial.print(animation);
        Serial.println("\" unknown.");
      }
    } else if (command.equals("stop")) {
      playing_animation = false;
    }
  }

  if (playing_animation && (millis() - last_animation_frame >= SERVO_UPDATE_INTERVAL)) {
    now = millis() * TIME_SCALE;

    if (current_animation.equals("xline")) {
      gimbal.write(sin(now)*6.0,0); // radius = 6
    }

    if (current_animation.equals("yline")) {
      gimbal.write(0,sin(now)*6.0); // radius = 6
    }

    last_animation_frame = millis();
  }

}

void printpos() {
  Serial.print("(");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.println(")");
}
