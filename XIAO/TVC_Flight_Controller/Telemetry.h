#ifndef Telemetry_h
#define Telemetry_h

#include <Arduino.h>
#include "config.h"

class TelemetryClass {
public:
  TelemetryClass();
  int log(float t, float p, float r, float pid_p, float pid_r, float alt);
  int print_logs();

private:
  int _current_entry;
  float _logs[TELEMETRY_HZ*TELEMETRY_RUNTIME_S][TELEMETRY_MEASUREMENTS];
};

#endif
