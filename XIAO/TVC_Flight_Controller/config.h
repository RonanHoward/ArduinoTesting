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
#define SERVO_PITCH_PIN    D7     // gimbal servo for the PITCH axis
#define SERVO_ROLL_PIN     D6    // gimbal servo for the ROLL axis
#define SERVO_CHUTE_PIN    D10    // parachute release servo

// ---------------------------------------------------------------------
//  2. SERVO CALIBRATION   -   centering (trim) and travel
//     CENTER_US: the pulse width (microseconds) that points the motor
//     mount straight ahead. Tune each until the gimbal is centered at
//     rest. US_PER_DEG: how many microseconds move the MOTOR one degree
//     (measure on the bench; it already includes any linkage ratio).
// ---------------------------------------------------------------------
#define SERVO_PITCH_CENTER_DEG  90
#define SERVO_ROLL_CENTER_DEG   90
#define SERVO_US_PER_DEG        10.3f   // microseconds per degree of motor deflection
#define TVC_MAX_DEFLECT_DEG     10.0f   // hard mechanical limit; commands are clamped
// vector cmd -> servo cmd gains
#define SERVO_ROLL_GAIN         1.0f
#define SERVO_PITCH_GAIN        1.0f

//  DIRECTION SIGNS
#define SERVO_PITCH_SIGN   (-1)
#define SERVO_ROLL_SIGN    (+1)

//  PARACHUTE servo positions (microseconds)
#define CHUTE_STOWED_US    1000    // holds the chute closed on the pad
#define CHUTE_DEPLOYED_US  2000    // releases the chute

// ---------------------------------------------------------------------
//  3. PID GAINS
//     One controller per CONTROLLABLE axis. A 2-servo gimbal can steer
//     PITCH and ROLL (tilt) only. YAW cannot be corrected by thrust
//     vectoring, so there is intentionally no roll controller.
//     Tune order: raise KP until it responds, add KD to damp wobble,
//     add a little KI last to remove a steady lean.
// ---------------------------------------------------------------------
#define PITCH_KP   0.40f
#define PITCH_KI   0.00f
#define PITCH_KD   0.1f

#define ROLL_KP     0.40f
#define ROLL_KI     0.00f
#define ROLL_KD     0.1f

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
#define LAUNCH_PIN                A2
#define LAUNCH_THRESHOLD_COUNTS   400

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
#define ROLL_AXIS_SIGN    (+1)

// ---------------------------------------------------------------------
//  7. TELEMETRY  (serial print for debugging / bench testing)
// ---------------------------------------------------------------------
#define TELEMETRY_ENABLED   1
#define TELEMETRY_HZ        10

// Additions as of August 4, 2026
// feedback
#define SPEAKER_PIN D8
#define SPEAKER_TONE_FREQ 2730

// For early flight purposes:
// How long the controller assumes the rocket is ascending (seconds)
#define DEBUG_ASCENT_TIME_S 50

#define TELEMETRY_LOG_FREQ_HZ 10

#endif // CONFIG_H
