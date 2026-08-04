#include "Telemetry.h"

TelemetryClass::TelemetryClass() {
  _current_entry = 0;
}

// TODO: array parameter for the love of god
int TelemetryClass::log(float t, float p, float r, float pid_p, float pid_r, float alt) {
  if (_current_entry >= TELEMETRY_HZ*TELEMETRY_RUNTIME_S) {
    _logs[_current_entry][0] = t;
    _logs[_current_entry][1] = p;
    _logs[_current_entry][2] = r;
    _logs[_current_entry][3] = pid_p;
    _logs[_current_entry][4] = pid_r;
    _logs[_current_entry][5] = alt;
    _current_entry++;
  }
}

int TelemetryClass::print_logs() {
  for (int i = 0; i < TELEMETRY_HZ*TELEMETRY_RUNTIME_S; i++) {
    Serial.print(_logs[i][0]);
    Serial.print(',');
    Serial.print(_logs[i][1]);
    Serial.print(',');
    Serial.print(_logs[i][2]);
    Serial.print(',');
    Serial.print(_logs[i][3]);
    Serial.print(',');
    Serial.print(_logs[i][4]);
    Serial.print(',');
    Serial.println(_logs[i][5]);
  }
  Serial.println();
}
