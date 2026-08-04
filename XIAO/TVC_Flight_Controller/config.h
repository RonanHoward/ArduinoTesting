// =====================================================================
//  TVC ROCKET FLIGHT CONTROLLER  -  CONFIGURATION
//
//  This is the ONLY file you normally need to edit. Every value you might
//  want to change to tune your rocket is here, grouped and commented.
//  You shouldn't have to open any other file to fly.
// =====================================================================
#ifndef CONFIG_H
#define CONFIG_H

// ---------------------------------------------------------------------
//  1. SERVO WIRING   -   which board pin each servo signal wire uses
// ---------------------------------------------------------------------
#define SERVO_PITCH_PIN    1      // gimbal servo for the PITCH axis
#define SERVO_YAW_PIN      2      // gimbal servo for the YAW axis
#define SERVO_CHUTE_PIN    3      // parachute release servo

// ---------------------------------------------------------------------
//  2. SERVO CALIBRATION   -   centering (trim) and travel
//     CENTER_US: the pulse width (microseconds) that points the motor
//     mount straight ahead. Tune each until the gimbal is centered at
//     rest. US_PER_DEG: how many microseconds move the MOTOR one degree
//     (measure on the bench; it already includes any linkage ratio).
// ---------------------------------------------------------------------
#define SERVO_PITCH_CENTER_US   1500
#define SERVO_YAW_CENTER_US     1500
#define SERVO_US_PER_DEG        10.0f   // microseconds per degree of motor deflection
#define TVC_MAX_DEFLECT_DEG     7.0f    // hard mechanical limit; commands are clamped

//  DIRECTION SIGNS. During the bench test (README step 3), if an axis
//  steers the WRONG way, flip its sign here from +1 to -1.
#define SERVO_PITCH_SIGN   (+1)
#define SERVO_YAW_SIGN     (+1)

//  PARACHUTE servo positions (microseconds).
#define CHUTE_STOWED_US    1000    // holds the chute closed on the pad
#define CHUTE_DEPLOYED_US  2000    // releases the chute

// ---------------------------------------------------------------------
//  3. PID GAINS   -   the heart of tuning. START SMALL.
//     One controller per CONTROLLABLE axis. A 2-servo gimbal can steer
//     PITCH and YAW (tilt) only. ROLL cannot be corrected by thrust
//     vectoring, so there is intentionally no roll controller.
//     Tune order: raise KP until it responds, add KD to damp wobble,
//     add a little KI last to remove a steady lean.
// ---------------------------------------------------------------------
#define PITCH_KP   0.20f
#define PITCH_KI   0.00f
#define PITCH_KD   0.05f

#define YAW_KP     0.20f
#define YAW_KI     0.00f
#define YAW_KD     0.05f

//  Largest vector angle the PID is allowed to ask for (degrees).
//  Keep this at or below the gimbal's mechanical limit.
#define PID_OUTPUT_LIMIT_DEG   TVC_MAX_DEFLECT_DEG

// ---------------------------------------------------------------------
//  4. LAUNCH DETECTION
//     For now: an analog pin rising above a threshold. (Planned: replace
//     with an accelerometer trigger - see detectLaunch() in
//     FlightController.h, the one place you'd change.)
//     Threshold is a raw analogRead count. To convert from volts:
//        counts = volts / ADC_reference_volts * ADC_max_counts
// ---------------------------------------------------------------------
#define LAUNCH_PIN                A0
#define LAUNCH_THRESHOLD_COUNTS   512

// ---------------------------------------------------------------------
//  5. PARACHUTE / DESCENT DETECTION
//     Apogee is found from the barometer: altitude climbs, then falls.
//     Deploy once altitude has dropped DESCENT_DROP_M below the highest
//     value seen, confirmed over several samples so noise can't trigger
//     it. A backup timer ALWAYS deploys as a failsafe.
// ---------------------------------------------------------------------
#define DESCENT_DROP_M           2.0f   // meters below apogee -> deploy
#define DESCENT_CONFIRM_SAMPLES  5      // consecutive descending samples
#define BACKUP_APOGEE_TIME_MS    8000   // failsafe deploy time after launch
                                        // (set a bit above your time-to-apogee)

// ---------------------------------------------------------------------
//  6. GYRO ESTIMATOR
// ---------------------------------------------------------------------
#define GYRO_BIAS_SAMPLES   500    // still samples averaged on the pad

//  Advanced: if the flight computer is mounted so a control axis reads
//  reversed, flip these instead of rewiring. Try the servo signs first.
#define PITCH_AXIS_SIGN   (+1)
#define YAW_AXIS_SIGN     (+1)

// ---------------------------------------------------------------------
//  7. TELEMETRY  (serial print for debugging / bench testing)
// ---------------------------------------------------------------------
#define TELEMETRY_ENABLED   1
#define TELEMETRY_HZ        20

#endif // CONFIG_H
