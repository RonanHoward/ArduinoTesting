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

## First-flight checklist

1. **Wire it and set the pins.** In `config.h`, set `SERVO_PITCH_PIN`,
   `SERVO_YAW_PIN`, `SERVO_CHUTE_PIN`, and `LAUNCH_PIN` to match your board.
2. **Center the gimbal (trim).** With the motor mount installed, adjust
   `SERVO_*_CENTER_US` until the mount points straight when idle.
3. **Check directions on the bench (critical).** Power up, trigger launch
   detection, and tilt the rocket by hand. The motor must swing to *oppose*
   the tilt. If an axis pushes the wrong way, flip `SERVO_PITCH_SIGN` or
   `SERVO_YAW_SIGN`. Getting this wrong makes the rocket steer itself into the
   ground — always verify before flying.
4. **Tune PID gently.** Start with the provided small gains. Raise `KP` until it
   responds firmly, add `KD` to stop oscillation, add a little `KI` last only if
   it leans. Bench-test on a gimbal test stand first.
5. **Set the recovery timing.** `BACKUP_APOGEE_TIME_MS` should be a bit longer
   than your expected time to apogee — it deploys the chute no matter what, as a
   failsafe if the barometer misses.

## Safety notes

- The parachute has a **backup timer** in addition to barometric detection, so a
  bad pressure reading can't leave you without a chute.
- On a sensor failure the controller enters an ERROR state, keeps the gimbal
  centered, and keeps the chute stowed rather than doing something unpredictable.
- The board does **not** wait forever for a USB serial connection, so it still
  boots when flying untethered.
- TVC currently runs the whole way to apogee. Ideally it would stop at motor
  burnout; detecting burnout cleanly needs the accelerometer, which is planned
  (the same upgrade that will replace the launch-detect pin).

## How launch detection works (and the planned change)

Right now launch is detected when `LAUNCH_PIN` rises above
`LAUNCH_THRESHOLD_COUNTS`. This is isolated in one small function
(`detectLaunch()` in `FlightController.h`) so it can later be swapped for an
accelerometer trigger without touching the rest of the code.
