#include <Servo.h>

Servo X;
Servo Y;

int px = 90;
int py = 90;

String command;

void setup() {

  Serial.begin(9600);

  delay(1000);

  X.attach(D4);
  Y.attach(D5);

  pinMode(D3, OUTPUT);
  digitalWrite(D3, HIGH);

  X.write(px);
  Y.write(py);

  Serial.println("Started. Enter a command.");
  Serial.print("(");
  Serial.print(px);
  Serial.print(",");
  Serial.print(py);
  Serial.println(")");

}

void loop() {

  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("x")) {
      px = command.substring(2).toInt();
      X.write(px);
      Serial.print("(");
      Serial.print(px);
      Serial.print(",");
      Serial.print(py);
      Serial.println(")");
    } else if (command.startsWith("y")) {
      py = command.substring(2).toInt();
      Y.write(py);
      Serial.print("(");
      Serial.print(px);
      Serial.print(",");
      Serial.print(py);
      Serial.println(")");
    }
  }

}
