# Revisions for flight 6

- EKF Updates at 10Hz rather than 100Hz
  - EKF still predicts at 100Hz; driven by the data-ready flag on the gyroscope
- Telemetry now logs
  1. phi_rad
  2. theta_rad
  3. pid_X u_n
  4. pid_y u_n
  5. P00
  6. P01
  7. P10
  8. P11

