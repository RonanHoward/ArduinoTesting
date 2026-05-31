#include "Telemetry.h"

Telemetry::Telemetry() {
  current_slot = 0;
}

void Telemetry::log(float time, float l1, float l2, float l3, float l4, float l5, float l6, float l7, float l8) {

  if (current_slot >= TELEMETRY_SAMPLES_PER_SEC*TELEMETRY_LOG_TIME_SEC) {
    return;
  }

  logs[current_slot][0] = time;
  logs[current_slot][1] = l1;
  logs[current_slot][2] = l2;
  logs[current_slot][3] = l3;
  logs[current_slot][4] = l4;
  logs[current_slot][5] = l5;
  logs[current_slot][6] = l6;
  logs[current_slot][7] = l7;
  logs[current_slot][8] = l8;

  current_slot++;

}
