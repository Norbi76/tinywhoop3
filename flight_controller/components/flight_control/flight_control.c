#include "flight_control.h"
#include "pid_controller.h"
#include "motor_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "FLIGHT_CTRL";

// ---------------------------------------------------------------------------
// Loop rates. The inner loop runs on every fc_task tick; the others are derived from it with
// divider counters so that all three stay phase-locked and no loop can drift relative to
// another.
// ---------------------------------------------------------------------------
#define INNER_LOOP_HZ 1000
#define MID_LOOP_DIVIDER   4    // 1000 / 4  = 250 Hz angle loop
#define OUTER_LOOP_DIVIDER 20   // 1000 / 20 =  50 Hz altitude and velocity loops

#define MID_LOOP_DT   ((float)MID_LOOP_DIVIDER / (float)INNER_LOOP_HZ)     // 0.004 s
#define OUTER_LOOP_DT ((float)OUTER_LOOP_DIVIDER / (float)INNER_LOOP_HZ)   // 0.020 s

// ---------------------------------------------------------------------------
// SAFETY CONSTANTS
// ---------------------------------------------------------------------------

// Control-link watchdog. No valid control frame inside this window and we disarm.
// The telemetry module sends at a fixed 50 Hz (20 ms), so 300 ms tolerates 15 lost frames
// before acting - long enough to ride out Wi-Fi hiccups, short enough to matter.
#define LINK_TIMEOUT_US 300000

// Arming gate: the throttle stick must be genuinely down, not just low.
#define ARM_THROTTLE_THRESHOLD 0.02f

// Below this throttle the motors are held off even when armed. Prevents the integrators
// winding up while the drone sits on the floor with the props barely turning.
#define MOTOR_IDLE_THRESHOLD 0.05f

// ---------------------------------------------------------------------------
// TODO(bench): HOVER_THROTTLE - MEASURE THIS.
// The normalised throttle at which the drone holds altitude with no vertical acceleration.
// The altitude loop's output is added on top of this, so a bad value means the altitude loop
// spends its whole range correcting a constant offset instead of doing useful work.
//
// How to measure: fly in angle mode, trim the throttle until the drone neither climbs nor
// sinks, and read the throttle value off the dashboard. Expect somewhere around 0.35-0.55
// for a 1S whoop, but it depends entirely on your all-up weight with the camera fitted.
// ---------------------------------------------------------------------------
#define HOVER_THROTTLE 0.45f

// Authority limits. These bound how hard each loop is allowed to push the one below it.
#define MAX_ANGLE_SETPOINT_DEG 25.0f    // outer velocity loop cannot ask for more tilt than this
#define MAX_RATE_SETPOINT_DPS 250.0f    // angle loop cannot ask for more rotation than this
#define MAX_ALTITUDE_THROTTLE_ADJUST 0.25f
#define MAX_CLIMB_RATE_MS 0.5f          // how fast the throttle stick moves the altitude setpoint

// ---------------------------------------------------------------------------
// TODO(tune): DEFAULT PID GAINS - ALL OF THESE ARE STARTING POINTS, NOT ANSWERS.
// Tune from the inside out with the props off first, then on a tether:
//   1. rate loops  (P until it oscillates, back off ~30%, then add D, then a little I)
//   2. angle loops (P only to begin with)
//   3. velocity and altitude loops last, and only once the inner two are solid
// Every one of these is live-tunable from the dashboard, which is the whole point of the
// gains message.
// ---------------------------------------------------------------------------
#define DEFAULT_RATE_KP 0.0020f
#define DEFAULT_RATE_KI 0.0100f
#define DEFAULT_RATE_KD 0.00004f

#define DEFAULT_ANGLE_KP 4.5f
#define DEFAULT_ANGLE_KI 0.0f
#define DEFAULT_ANGLE_KD 0.0f

#define DEFAULT_VELOCITY_KP 8.0f
#define DEFAULT_VELOCITY_KI 1.5f
#define DEFAULT_VELOCITY_KD 0.0f

#define DEFAULT_ALTITUDE_KP 0.40f
#define DEFAULT_ALTITUDE_KI 0.15f
#define DEFAULT_ALTITUDE_KD 0.05f

// Derivative filter cutoffs. The rate loop sees raw gyro and needs the most filtering.
#define RATE_D_CUTOFF_HZ 60.0f
#define ANGLE_D_CUTOFF_HZ 30.0f
#define OUTER_D_CUTOFF_HZ 10.0f

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static pid_controller_t pid_loops[TELEMETRY_LOOP_COUNT];

static flight_state_t flight_state = FLIGHT_STATE_DISARMED;
static bool kill_latched;

// Latest control input, written by the telemetry task and read by fc_task.
static telemetry_control_payload_t control_input;
static int64_t last_control_input_us;
static portMUX_TYPE control_input_spinlock = portMUX_INITIALIZER_UNLOCKED;

// Divider counters for the mid and outer loops.
static uint32_t tick_counter;

// Setpoints handed down between cascade levels. The mid loop writes the rate setpoints that
// the inner loop consumes; the outer loop writes the angle setpoints the mid loop consumes.
static float rate_setpoint_roll, rate_setpoint_pitch, rate_setpoint_yaw;
static float angle_setpoint_roll, angle_setpoint_pitch;
static float throttle_command;

// Altitude hold target, captured when the mode engages.
static float altitude_setpoint;
static bool altitude_hold_engaged;

// Last motor commands, for telemetry.
static float motor_output[MOTOR_COUNT];

// Measured inner loop rate, for verifying on the bench that this really is 1 kHz.
static uint16_t measured_loop_hz;
static int64_t loop_rate_window_start_us;
static uint32_t loop_rate_counter;

// Cached status bits.
static bool link_ok;

static float clampf(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// Clears every integrator and derivative history in the cascade.
// Called on BOTH arm and disarm. On disarm because whatever the integrators wound up to while
// the drone was being wrestled to the ground is meaningless; on arm because time may have
// passed since the disarm and because it is the last chance to guarantee a clean start.
static void flight_control_reset_all_pids(void) {
    for (int i = 0; i < TELEMETRY_LOOP_COUNT; i++) {
        pid_reset(&pid_loops[i]);
    }

    rate_setpoint_roll = 0.0f;
    rate_setpoint_pitch = 0.0f;
    rate_setpoint_yaw = 0.0f;
    angle_setpoint_roll = 0.0f;
    angle_setpoint_pitch = 0.0f;
    throttle_command = 0.0f;
    altitude_hold_engaged = false;
}

esp_err_t flight_control_init(void) {
    esp_err_t error;

    // --- Inner rate loops: gyro rate (deg/s) -> normalised motor mix contribution ---
    // Output range is deliberately narrow: at +/-0.5 a single axis can never claim more than
    // half the motor range, which leaves room for the other axes and the throttle.
    for (int i = TELEMETRY_LOOP_RATE_ROLL; i <= TELEMETRY_LOOP_RATE_YAW; i++) {
        error = pid_init(&pid_loops[i],
                         DEFAULT_RATE_KP, DEFAULT_RATE_KI, DEFAULT_RATE_KD,
                         -0.5f, 0.5f,
                         0.2f,
                         RATE_D_CUTOFF_HZ,
                         1.0f / (float)INNER_LOOP_HZ);
        if (error != ESP_OK) {
            return error;
        }
    }

    // --- Mid angle loops: angle error (deg) -> rate setpoint (deg/s) ---
    for (int i = TELEMETRY_LOOP_ANGLE_ROLL; i <= TELEMETRY_LOOP_ANGLE_PITCH; i++) {
        error = pid_init(&pid_loops[i],
                         DEFAULT_ANGLE_KP, DEFAULT_ANGLE_KI, DEFAULT_ANGLE_KD,
                         -MAX_RATE_SETPOINT_DPS, MAX_RATE_SETPOINT_DPS,
                         50.0f,
                         ANGLE_D_CUTOFF_HZ,
                         MID_LOOP_DT);
        if (error != ESP_OK) {
            return error;
        }
    }

    // --- Outer velocity loops: velocity error (m/s) -> angle setpoint (deg) ---
    for (int i = TELEMETRY_LOOP_VEL_X; i <= TELEMETRY_LOOP_VEL_Y; i++) {
        error = pid_init(&pid_loops[i],
                         DEFAULT_VELOCITY_KP, DEFAULT_VELOCITY_KI, DEFAULT_VELOCITY_KD,
                         -MAX_ANGLE_SETPOINT_DEG, MAX_ANGLE_SETPOINT_DEG,
                         10.0f,
                         OUTER_D_CUTOFF_HZ,
                         OUTER_LOOP_DT);
        if (error != ESP_OK) {
            return error;
        }
    }

    // --- Outer altitude loop: altitude error (m) -> throttle adjustment ---
    error = pid_init(&pid_loops[TELEMETRY_LOOP_ALTITUDE],
                     DEFAULT_ALTITUDE_KP, DEFAULT_ALTITUDE_KI, DEFAULT_ALTITUDE_KD,
                     -MAX_ALTITUDE_THROTTLE_ADJUST, MAX_ALTITUDE_THROTTLE_ADJUST,
                     MAX_ALTITUDE_THROTTLE_ADJUST,
                     OUTER_D_CUTOFF_HZ,
                     OUTER_LOOP_DT);
    if (error != ESP_OK) {
        return error;
    }

    flight_state = FLIGHT_STATE_DISARMED;
    kill_latched = false;
    link_ok = false;
    memset(&control_input, 0, sizeof(control_input));
    memset(motor_output, 0, sizeof(motor_output));
    flight_control_reset_all_pids();

    ESP_LOGI(TAG, "Flight control ready: %d Hz inner, %d Hz mid, %d Hz outer",
             INNER_LOOP_HZ, INNER_LOOP_HZ / MID_LOOP_DIVIDER, INNER_LOOP_HZ / OUTER_LOOP_DIVIDER);

    return ESP_OK;
}

void flight_control_set_control_input(const telemetry_control_payload_t *control) {
    if (control == NULL) {
        return;
    }

    // Short critical section: this runs on core 0 while fc_task reads on core 1, and a torn
    // read here would mean acting on half of one frame and half of another.
    taskENTER_CRITICAL(&control_input_spinlock);
    control_input = *control;
    last_control_input_us = esp_timer_get_time();
    taskEXIT_CRITICAL(&control_input_spinlock);
}

// ---------------------------------------------------------------------------
// X-QUAD MIXER
//
// >>> EVERY SIGN IN THIS FUNCTION NEEDS BENCH VERIFICATION. <<<
// This is the single most likely place for the drone to flip on its first flight. Verify it
// with the PROPS OFF, by arming, nudging each axis, and watching which motors speed up.
//
// Motor layout (see motor_driver.h):
//        FRONT
//   M0 ........ M1        M0 front-left  (CW)    M1 front-right (CCW)
//   M2 ........ M3        M2 rear-left   (CCW)   M3 rear-right  (CW)
//        REAR
//
// ROLL  (+ = roll right, right side down): lift the LEFT side  -> M0, M2 up / M1, M3 down
// PITCH (+ = nose up):                     lift the REAR       -> M2, M3 up / M0, M1 down
// YAW   (+ = nose right): a motor spinning CW drags the frame CCW, so to rotate the frame
//                         CW (nose right) we speed up the CCW motors -> M1, M2 up / M0, M3 down
//
// Bench checks, props OFF, armed, a little throttle:
//   command ROLL RIGHT  -> M0 and M2 must speed up
//   command PITCH UP    -> M2 and M3 must speed up
//   command YAW RIGHT   -> M1 and M2 must speed up
// If an axis is backwards, flip the sign of that whole column here.
// If the prop directions on your build differ from the CW/CCW above, the YAW column is the
// one that changes.
// ---------------------------------------------------------------------------
static void flight_control_mix(float throttle, float roll, float pitch, float yaw, float out[MOTOR_COUNT]) {
    out[MOTOR_FRONT_LEFT]  = throttle + roll - pitch - yaw;
    out[MOTOR_FRONT_RIGHT] = throttle - roll - pitch + yaw;
    out[MOTOR_REAR_LEFT]   = throttle + roll + pitch + yaw;
    out[MOTOR_REAR_RIGHT]  = throttle - roll + pitch - yaw;

    // --- Proportional saturation --------------------------------------------
    // If any motor is asking for more than full thrust, scale ALL FOUR down by the same
    // factor. Simply clipping the one that overflowed would silently change the ratios
    // between motors, which is exactly the same as corrupting the attitude command - the
    // drone would roll or pitch when it was told to do neither. Scaling proportionally
    // costs some total thrust but preserves the attitude the controller asked for.
    float maximum = out[0];
    for (int i = 1; i < MOTOR_COUNT; i++) {
        if (out[i] > maximum) {
            maximum = out[i];
        }
    }

    if (maximum > 1.0f) {
        const float scale = 1.0f / maximum;
        for (int i = 0; i < MOTOR_COUNT; i++) {
            out[i] *= scale;
        }
    }

    // Negative demands cannot be delivered by a unidirectional brushed motor.
    for (int i = 0; i < MOTOR_COUNT; i++) {
        out[i] = clampf(out[i], 0.0f, 1.0f);
    }
}

// Evaluates the arming gate and manages state transitions.
// Returns true if the motors may run this iteration.
static bool flight_control_update_arming(const telemetry_control_payload_t *control,
                                         const attitude_state_t *attitude) {
    const bool kill_requested = (control->flags & TELEMETRY_CTRL_FLAG_KILL) != 0;
    const bool arm_requested = (control->armed != 0);

    // --- Kill is absolute and latching --------------------------------------
    if (kill_requested) {
        if (!kill_latched) {
            ESP_LOGW(TAG, "KILL received - motors cut and latched");
        }
        kill_latched = true;
    }

    if (kill_latched) {
        if (flight_state != FLIGHT_STATE_KILLED) {
            flight_state = FLIGHT_STATE_KILLED;
            flight_control_reset_all_pids();
        }

        // The latch only clears when the pilot has released kill AND is no longer asking to
        // arm - so letting go of the kill button cannot by itself spin the motors back up.
        if (!kill_requested && !arm_requested) {
            kill_latched = false;
            flight_state = FLIGHT_STATE_DISARMED;
            ESP_LOGI(TAG, "Kill latch cleared, back to DISARMED");
        }

        return false;
    }

    // --- Link watchdog -------------------------------------------------------
    if (!link_ok) {
        if (flight_state == FLIGHT_STATE_ARMED) {
            ESP_LOGW(TAG, "Control link lost - disarming");
            flight_state = FLIGHT_STATE_DISARMED;
            flight_control_reset_all_pids();
        }
        return false;
    }

    // --- Disarm request ------------------------------------------------------
    if (!arm_requested) {
        if (flight_state == FLIGHT_STATE_ARMED) {
            ESP_LOGI(TAG, "Disarmed by pilot");
            flight_state = FLIGHT_STATE_DISARMED;
            // Integrators cleared on disarm as well as arm - see flight_control_reset_all_pids().
            flight_control_reset_all_pids();
        }
        return false;
    }

    // --- Arming gate ---------------------------------------------------------
    // All four conditions must hold. This is the only path into FLIGHT_STATE_ARMED.
    if (flight_state != FLIGHT_STATE_ARMED) {
        const bool attitude_ready = attitude->initialized;
        const bool throttle_down = (control->throttle < ARM_THROTTLE_THRESHOLD);

        if (!attitude_ready) {
            // Logged at debug level because this is normal for the first half second after boot.
            return false;
        }
        if (!throttle_down) {
            // Refusing to arm on a raised throttle stick is what stops the drone leaping off
            // the bench the instant it is armed.
            return false;
        }

        ESP_LOGI(TAG, "ARMED (throttle %.3f, attitude ready, link ok)", (double)control->throttle);
        flight_control_reset_all_pids();
        flight_state = FLIGHT_STATE_ARMED;
    }

    return true;
}

// Outer loop at 50 Hz: altitude -> throttle, body velocity -> angle setpoints.
static void flight_control_outer_loop(const telemetry_control_payload_t *control,
                                      const nav_state_t *nav) {
    const bool wants_altitude_hold = (control->flight_mode == TELEMETRY_MODE_ALT_HOLD ||
                                      control->flight_mode == TELEMETRY_MODE_POS_HOLD ||
                                      (control->flags & TELEMETRY_CTRL_FLAG_HOLD) != 0);
    const bool wants_position_hold = (control->flight_mode == TELEMETRY_MODE_POS_HOLD ||
                                      (control->flags & TELEMETRY_CTRL_FLAG_HOLD) != 0);

    // --- Altitude ------------------------------------------------------------
    // Disengages the moment the ToF stops being trustworthy. The pilot's throttle takes over
    // directly, which is a mode change they will feel, but it is far better than holding an
    // altitude derived from a bad range.
    if (wants_altitude_hold && nav != NULL && nav->altitude_valid) {
        if (!altitude_hold_engaged) {
            // Capture the current height as the target on engagement, and start the loop from
            // a clean integrator so it does not inherit anything from a previous engagement.
            altitude_setpoint = nav->altitude;
            altitude_hold_engaged = true;
            pid_reset(&pid_loops[TELEMETRY_LOOP_ALTITUDE]);
            ESP_LOGI(TAG, "Altitude hold engaged at %.2f m", (double)altitude_setpoint);
        }

        // The throttle stick becomes a climb/descend command around centre rather than a
        // direct thrust command.
        const float stick_offset = control->throttle - 0.5f;
        if (fabsf(stick_offset) > 0.1f) {
            altitude_setpoint += stick_offset * 2.0f * MAX_CLIMB_RATE_MS * OUTER_LOOP_DT;
            altitude_setpoint = clampf(altitude_setpoint, 0.05f, 1.20f);
        }

        const float adjustment = pid_update(&pid_loops[TELEMETRY_LOOP_ALTITUDE],
                                            altitude_setpoint, nav->altitude, OUTER_LOOP_DT);
        throttle_command = clampf(HOVER_THROTTLE + adjustment, 0.0f, 1.0f);
    } else {
        if (altitude_hold_engaged) {
            ESP_LOGW(TAG, "Altitude hold disengaged (altitude no longer valid)");
            altitude_hold_engaged = false;
        }
        // Straight pass-through of the pilot's throttle.
        throttle_command = clampf(control->throttle, 0.0f, 1.0f);
    }

    // --- Body velocity -------------------------------------------------------
    if (wants_position_hold && nav != NULL && nav->velocity_valid) {
        // The roll/pitch sticks command a VELOCITY here rather than an angle, so centring
        // them asks for zero velocity, which is what makes the drone hold station.
        // Scale: full stick deflection (+/-1 after the angle mapping) asks for 1 m/s.
        const float velocity_setpoint_x = (control->pitch_setpoint / MAX_ANGLE_SETPOINT_DEG) * 1.0f;
        const float velocity_setpoint_y = (control->roll_setpoint / MAX_ANGLE_SETPOINT_DEG) * 1.0f;

        // Forward velocity error is corrected by pitching. Positive pitch is nose UP, which
        // moves the drone BACKWARD, hence the negation.
        // SIGN NEEDS BENCH VERIFICATION - if the drone runs away instead of holding station,
        // this is the first thing to flip.
        angle_setpoint_pitch = -pid_update(&pid_loops[TELEMETRY_LOOP_VEL_X],
                                           velocity_setpoint_x, nav->velocity_x, OUTER_LOOP_DT);
        // Right velocity error is corrected by rolling right, so no negation here.
        angle_setpoint_roll = pid_update(&pid_loops[TELEMETRY_LOOP_VEL_Y],
                                         velocity_setpoint_y, nav->velocity_y, OUTER_LOOP_DT);
    } else {
        // Angle mode: the sticks are the angle setpoints directly.
        angle_setpoint_roll = control->roll_setpoint;
        angle_setpoint_pitch = control->pitch_setpoint;
    }

    angle_setpoint_roll = clampf(angle_setpoint_roll, -MAX_ANGLE_SETPOINT_DEG, MAX_ANGLE_SETPOINT_DEG);
    angle_setpoint_pitch = clampf(angle_setpoint_pitch, -MAX_ANGLE_SETPOINT_DEG, MAX_ANGLE_SETPOINT_DEG);
}

// Mid loop at 250 Hz: angle error -> rate setpoints.
static void flight_control_mid_loop(const telemetry_control_payload_t *control,
                                    const attitude_state_t *attitude) {
    rate_setpoint_roll = pid_update(&pid_loops[TELEMETRY_LOOP_ANGLE_ROLL],
                                    angle_setpoint_roll, attitude->roll, MID_LOOP_DT);
    rate_setpoint_pitch = pid_update(&pid_loops[TELEMETRY_LOOP_ANGLE_PITCH],
                                     angle_setpoint_pitch, attitude->pitch, MID_LOOP_DT);

    // Yaw has no angle loop - there is no magnetometer, so absolute heading is not observable.
    // The stick commands a yaw RATE directly and the inner loop tracks it.
    rate_setpoint_yaw = clampf(control->yaw_setpoint, -MAX_RATE_SETPOINT_DPS, MAX_RATE_SETPOINT_DPS);
}

// Inner loop at 1 kHz: rate error -> mixer -> motors.
static void flight_control_inner_loop(const attitude_state_t *attitude, float dt) {
    const float roll_output = pid_update(&pid_loops[TELEMETRY_LOOP_RATE_ROLL],
                                         rate_setpoint_roll, attitude->roll_rate, dt);
    const float pitch_output = pid_update(&pid_loops[TELEMETRY_LOOP_RATE_PITCH],
                                          rate_setpoint_pitch, attitude->pitch_rate, dt);
    const float yaw_output = pid_update(&pid_loops[TELEMETRY_LOOP_RATE_YAW],
                                        rate_setpoint_yaw, attitude->yaw_rate, dt);

    flight_control_mix(throttle_command, roll_output, pitch_output, yaw_output, motor_output);

    esp_err_t error = motor_set_thrust_all(motor_output);
    if (error != ESP_OK) {
        // A failed motor write while armed is serious - log it, but do not spam at 1 kHz.
        static int64_t last_log_us;
        const int64_t now_us = esp_timer_get_time();
        if ((now_us - last_log_us) > 1000000) {
            ESP_LOGE(TAG, "motor_set_thrust_all failed: %s", esp_err_to_name(error));
            last_log_us = now_us;
        }
    }
}

// Forces motors off and zeroes the recorded outputs.
static void flight_control_stop_motors(void) {
    motor_all_stop();
    memset(motor_output, 0, sizeof(motor_output));
}

void flight_control_update(const attitude_state_t *attitude, const nav_state_t *nav, float dt) {
    if (attitude == NULL || dt <= 0.0f) {
        flight_control_stop_motors();
        return;
    }

    const int64_t now_us = esp_timer_get_time();

    // --- Measured loop rate, for bench verification ---------------------------
    loop_rate_counter++;
    if ((now_us - loop_rate_window_start_us) >= 1000000) {
        measured_loop_hz = (uint16_t)loop_rate_counter;
        loop_rate_counter = 0;
        loop_rate_window_start_us = now_us;
    }

    // --- Snapshot the control input ------------------------------------------
    telemetry_control_payload_t control;
    int64_t input_age_us;

    taskENTER_CRITICAL(&control_input_spinlock);
    control = control_input;
    input_age_us = now_us - last_control_input_us;
    taskEXIT_CRITICAL(&control_input_spinlock);

    // last_control_input_us == 0 means nothing has ever arrived, which is not "link ok".
    link_ok = (last_control_input_us != 0) && (input_age_us < LINK_TIMEOUT_US);

    // --- Arming --------------------------------------------------------------
    const bool motors_allowed = flight_control_update_arming(&control, attitude);

    if (!motors_allowed) {
        // MOTORS FORCED TO ZERO WHENEVER DISARMED. This is the single most important line in
        // the file: every path that does not explicitly authorise the motors ends up here.
        flight_control_stop_motors();
        tick_counter = 0;   // restart the dividers so the next arm begins on a clean phase
        return;
    }

    // --- Idle cutoff ---------------------------------------------------------
    // Armed but the throttle is on the floor: hold the motors off and keep the integrators
    // clear, so the controller cannot wind up against the ground before takeoff.
    if (control.throttle < MOTOR_IDLE_THRESHOLD && !altitude_hold_engaged) {
        flight_control_stop_motors();
        flight_control_reset_all_pids();
        return;
    }

    // --- Cascade -------------------------------------------------------------
    // Divider counters, so all three loops stay locked to the same 1 kHz timebase.
    tick_counter++;

    if ((tick_counter % OUTER_LOOP_DIVIDER) == 0) {
        flight_control_outer_loop(&control, nav);
    }

    if ((tick_counter % MID_LOOP_DIVIDER) == 0) {
        flight_control_mid_loop(&control, attitude);
    }

    flight_control_inner_loop(attitude, dt);
}

void flight_control_get_status(telemetry_status_payload_t *out) {
    if (out == NULL) {
        return;
    }

    memset(out, 0, sizeof(*out));

    attitude_state_t attitude;
    attitude_get(&attitude);

    nav_state_t nav;
    nav_estimator_get(&nav);

    out->roll = attitude.roll;
    out->pitch = attitude.pitch;
    out->yaw = attitude.yaw;

    out->altitude = nav.altitude;
    out->climb_rate = nav.climb_rate;
    out->velocity_x = nav.velocity_x;
    out->velocity_y = nav.velocity_y;

    // TODO(hardware): battery voltage. There is no ADC divider wired up yet, so this reports
    // the nominal value. Once you have a divider on the pack, read it here and also feed it
    // to motor_update_battery_voltage() so thrust compensation actually does something.
    out->battery_voltage = 3.8f;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        out->motor[i] = motor_output[i];
    }

    out->loop_hz = measured_loop_hz;
    out->armed = (flight_state == FLIGHT_STATE_ARMED) ? 1 : 0;
    out->flight_mode = control_input.flight_mode;

    out->flags = 0;
    if (link_ok)               out->flags |= TELEMETRY_STATUS_FLAG_LINK_OK;
    if (attitude.initialized)  out->flags |= TELEMETRY_STATUS_FLAG_ATTITUDE_INIT;
    if (nav.altitude_valid)    out->flags |= TELEMETRY_STATUS_FLAG_ALT_VALID;
    if (nav.velocity_valid)    out->flags |= TELEMETRY_STATUS_FLAG_VEL_VALID;
    if (kill_latched)          out->flags |= TELEMETRY_STATUS_FLAG_KILLED;
}

bool flight_control_is_armed(void) {
    return flight_state == FLIGHT_STATE_ARMED;
}

flight_state_t flight_control_get_state(void) {
    return flight_state;
}

esp_err_t flight_control_set_gains(telemetry_loop_id_t loop_id, float kp, float ki, float kd) {
    if ((int)loop_id < 0 || (int)loop_id >= TELEMETRY_LOOP_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    pid_set_gains(&pid_loops[loop_id], kp, ki, kd);
    ESP_LOGI(TAG, "Loop %d gains set: kp=%.5f ki=%.5f kd=%.5f",
             (int)loop_id, (double)kp, (double)ki, (double)kd);

    return ESP_OK;
}

void flight_control_set_rate_gains(float kp, float ki, float kd) {
    pid_set_gains(&pid_loops[TELEMETRY_LOOP_RATE_ROLL], kp, ki, kd);
    pid_set_gains(&pid_loops[TELEMETRY_LOOP_RATE_PITCH], kp, ki, kd);
}

void flight_control_set_angle_gains(float kp, float ki, float kd) {
    pid_set_gains(&pid_loops[TELEMETRY_LOOP_ANGLE_ROLL], kp, ki, kd);
    pid_set_gains(&pid_loops[TELEMETRY_LOOP_ANGLE_PITCH], kp, ki, kd);
}

void flight_control_set_velocity_gains(float kp, float ki, float kd) {
    pid_set_gains(&pid_loops[TELEMETRY_LOOP_VEL_X], kp, ki, kd);
    pid_set_gains(&pid_loops[TELEMETRY_LOOP_VEL_Y], kp, ki, kd);
}

void flight_control_set_altitude_gains(float kp, float ki, float kd) {
    pid_set_gains(&pid_loops[TELEMETRY_LOOP_ALTITUDE], kp, ki, kd);
}
