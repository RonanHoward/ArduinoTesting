# TVC7626 Flight Control (B)

1. Find orientation while stationary on pad.
  - Use accelerometer only, heavy IIR filter
2. Detect launch.
  - z-axis acceleration exceeds specified threshold
3. State estimate with gyroscope only.
  - Euler integration, basic offset compensation
