# TVC7626 Flight Control (B1-f7)

B1-f7: Control model B *pure gyroscope*, flight 7

### Detect Launch
Check if acceleration is greater than 1g. When the current accelerometer readings trigger launch, use the initial orientation calculated from previous accelerometer readings. The previous gyroscope rates must also be saved for accurate integration, since we will use the trapezoidal method.


### Initial Orientation
Since we use gyroscope state estimation, we need to estimate initial orientation $(\dot\phi_0,\dot\theta_0)$. Assuming stationary before launch,
$$
\begin{bmatrix}
    \dot\phi_0
    \\
    \dot\theta_0
\end{bmatrix}
=
\begin{bmatrix}
    \tan^{-1}\left(\frac{a_y}{a_z}\right)
    \\
    \sin^{-1}\left(a_x\right)
\end{bmatrix}
$$
Where $a_x$, $a_y$, and $a_z$ are in units $[g]=\left[9.81\frac m{s^2}\right]$.

### Navigation
Strategy is to numerically integrate the rate-gyro during flight. This program uses the trapezoidal method. As a sequence, this looks like
$$\dot\theta_{i}=\dot\theta_{i-1}+\frac{\omega_{i-1}+\omega_{i}}2\Delta t_i$$
for gyro reading $\omega_i$ after $\Delta t_i$ time elapsed from last reading.

### Notes
- Some tuning and testing needed
- This program uses `micros()` for time, which overloads after about 71 minutes. This is unaccounted for.
- TODO: Barometer on last telemetry slot.