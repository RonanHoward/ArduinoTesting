# TVC Rocket Flight Controller

Thrust-vector-control flight computer for a XIAO RA4M1 with an LSM6DSV gyro
and an LPS22HB barometer. It keeps the rocket vertical by gimballing the motor
with two servos, and releases a parachute with a third servo once the barometer
detects descent.

## The only file you normally edit: `config.h`

All tunable values — servo pins, servo trim, PID gains, launch threshold,
parachute settings — live in `config.h`. You should be able to build, tune, and
fly without opening any other file. Each setting is commented with what it does
and how to find the right value.

## File map

| File | What it is | Touch it? |
|------|------------|-----------|
| `config.h` | **All your settings.** Pins, trim, gains, thresholds. | **Yes — this is your file.** |
| `TVC_Flight_Controller.ino` | Top level. Just starts the controller. | Rarely |
| `FlightController.h` | The flight state machine (pad → ascent → descent → landed) and launch detection. | Advanced |
| `AttitudeEstimator.h` | Turns gyro data into a tilt estimate. | Advanced |
| `PID.h` | Generic PID controller used for pitch and yaw. | Advanced |
| `TVCMount.h` | Converts pitch/yaw commands into the two gimbal servo signals. | Advanced |
| `Parachute.h` | Altitude/apogee detection and the release servo. | Advanced |
| `lsm6dsv.h` / `.cpp` | Gyro driver. | No |
| `lps22hb.h` / `.cpp` | Barometer driver. | No |

Put **all** of these files in one folder named `TVC_Flight_Controller` (same
name as the `.ino`). The Arduino IDE compiles them together automatically.

## How launch detection works (and the planned change)

Right now launch is detected when `LAUNCH_PIN` rises above
`LAUNCH_THRESHOLD_COUNTS`. This is isolated in one small function
(`detectLaunch()` in `FlightController.h`) so it can later be swapped for an
accelerometer trigger without touching the rest of the code.
