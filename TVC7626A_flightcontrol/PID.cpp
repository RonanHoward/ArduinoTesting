#include "PID.h"

PID::PID() {
  // ...
}

void PID::update(float measurement) {
  e_n = measurement - control; // Radians
  p_n = Kp * e_n;

  i_n += (Ki * T / 2) * (e_n + e_n_1);
  d_n = d_n_1 * (2 * t - T) / (2 * t + T) + 2 * Kd / (2 * t + T) * (e_n - e_n_1);

  if (i_n > int_limit_max) {  // Integrator wind-up limits
    i_n = int_limit_max;
  }
  if (i_n < int_limit_min) {
    i_n = int_limit_min;
  }

  u_n = p_n + i_n + d_n;
  d_n_1 = d_n;
  e_n_1 = e_n;
}

float PID::get_un() {
  return u_n;
}

float PID::get_frequency() {
  return frequency;
}
