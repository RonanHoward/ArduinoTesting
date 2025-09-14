#include "Gimbal_PID.h"

Gimbal_PID::Gimbal_PID() {
  // ...
}

void Gimbal_PID::update(float measurement) {
  e_n = control - measurement; /*** Radians ***/
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

float Gimbal_PID::get_un() {
  return u_n;
}

float Gimbal_PID::get_frequency() {
  return frequency;
}
