#include "Telemetry.h"

Telemetry::Telemetry() {
  current_slot = 0;
}

void Telemetry::log(float time, float l1, float l2, float l3, float l4) {

  if (current_slot >= TELEMETRY_SAMPLES_PER_SEC*TELEMETRY_LOG_TIME_SEC) {
    return;
  }

  logs[current_slot][0] = time;
  logs[current_slot][1] = l1;
  logs[current_slot][2] = l2;
  logs[current_slot][3] = l3;
  logs[current_slot][4] = l4;

  current_slot++;

}
