#include <Servo.h>

Servo blue;
Servo black;

int blue_values[] = {-115,-75,-33,10,53,95,136,173,205,232,253,266,273,274,269,258,243,225,204,181,158,133,108,83,58,34,9,-16,-40,-65,-89,-114,-138,-162,-186,-208,-229,-248,-264,-277,-286,-291,-290,-285,-275,-260,-240,-215,-185,-152,-115};

int black_values[] = {-275,-262,-244,-221,-192,-160,-125,-87,-49,-10,28,64,99,130,159,185,207,226,241,253,261,267,269,267,263,256,246,234,218,200,180,157,132,105,76,46,15,-17,-50,-82,-114,-145,-174,-201,-226,-246,-263,-274,-280,-280,-275};

void setup() {

  blue.attach(D4);
  black.attach(D5);

  pinMode(D3, OUTPUT);
  digitalWrite(D3, HIGH);

}

int count = 0;

void loop() {
  
  blue.writeMicroseconds(1447 + blue_values[count]);
  black.writeMicroseconds(1447 + black_values[count]);

  count = count + 1;

  if ( count >= 50 ) {
    count = 0;
  }

  delay(20);

}
