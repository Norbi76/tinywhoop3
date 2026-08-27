#pragma once

#include <stdbool.h>
#include "esp_err.h"

// pid_controller.h - the generic PID primitive the whole control cascade is built from.
//
// WHAT THIS COMPONENT IS FOR
//   A plain scalar PID with no globals, no tasks, no ESP-IDF dependencies. All state lives in the
//   caller's pid_controller_t, which is what makes it safe to instantiate eight times.
//
// HOW IT DOES ITS JOB - four departures from the textbook formula, all deliberate
//   1. Derivative on MEASUREMENT, not error -> no motor kick when the pilot steps a stick.
//   2. Low-pass filtered derivative        -> prop vibration does not reach the motors as D.
//   3. Conditional-integration anti-windup -> a drone held on the bench does not wind up.
//   4. The integrator stores the ki-SCALED sum -> changing ki mid-flight does not jolt.
//   Each is explained where it is implemented in pid_controller.c, and in README.md.
//
// TWO RULES FOR CALLERS
//   - pid_reset() on EVERY arm and disarm. A stale integrator is the classic cause of a quad
//     flipping the instant it is armed.
//   - This component locks nothing. If gains must change from another task, go through
//     pid_registry's queue rather than writing the struct - that is what it is for.
//
// Generic PID controller.
// One implementation, instantiated 8 times by flight_control (3 rate loops, 2 angle loops,
// 2 velocity loops, 1 altitude loop). Nothing in here knows anything about quadcopters.
//
// Three properties matter for a flight controller and are all implemented below:
//
//  1. Derivative on measurement, not on error. A step change in the setpoint (pilot slams the
//     stick) would otherwise produce an infinite derivative and a violent motor kick.
//  2. Low-pass filtered derivative. Raw gyro differentiation amplifies frame vibration into
//     the D term, which is the usual cause of hot motors on a small quad.
//  3. Conditional-integration anti-windup. While the output is saturated, the integrator is
//     frozen in the direction that would push it further into saturation. Without this, a
//     drone held on the bench winds up its integrator and lurches when released.
typedef struct {
    float kp;
    float ki;
    float kd;

    // The integrator accumulates the ki-SCALED term (sum of ki * error * dt), not the raw
    // error integral. This matters for live tuning: changing ki mid-flight then only affects
    // future contributions instead of instantly rescaling everything accumulated so far,
    // which would show up as a sudden jolt.
    float integrator;
    float integrator_limit;

    float prev_measurement;
    float d_filtered;
    float d_alpha;           // derivative low-pass smoothing factor, 0..1

    float out_min;
    float out_max;

    // Retained so a gain change can recompute d_alpha, and so the live tuning UI can read the
    // cutoff back. d_alpha alone is not enough: recovering the cutoff from it needs the period,
    // and there is nowhere else the period is kept.
    float d_cutoff_hz;
    float nominal_dt;

    bool first_update;       // suppresses the derivative spike on the very first call

    // Retained for logging / bench inspection - lets you see which term is doing the work.
    float last_p;
    float last_i;
    float last_d;

    // Everything below is telemetry only. Nothing in the control law reads it back; it exists so
    // pid_registry can publish a complete picture of one iteration without the loop having to
    // hand it the setpoint and measurement separately.
    float last_setpoint;     // as passed in, i.e. injection offset already included
    float last_measurement;
    float last_output;       // post-clamp return value
    bool out_saturated;      // the clamp actually bit on the last iteration
    bool integrator_clamped; // the integrator was frozen by anti-windup or hit integrator_limit
} pid_controller_t;

// Initialises a PID instance and clears its state.
// @param pid Pointer to the controller to initialise.
// @param kp, ki, kd Initial gains.
// @param out_min, out_max Output saturation limits. Also drive the anti-windup logic.
// @param integrator_limit Hard clamp on the magnitude of the integrator term.
// @param d_cutoff_hz Cutoff frequency of the derivative low-pass filter, in Hz.
// @param nominal_dt Expected loop period in seconds, used to derive the filter coefficient.
// @return ESP_OK on success, ESP_ERR_INVALID_ARG on a NULL pointer or non-positive dt/cutoff.
esp_err_t pid_init(pid_controller_t *pid,
                   float kp, float ki, float kd,
                   float out_min, float out_max,
                   float integrator_limit,
                   float d_cutoff_hz,
                   float nominal_dt);

// Runs one PID iteration.
// @param pid Pointer to the controller.
// @param setpoint Desired value.
// @param measurement Current measured value, in the same units as the setpoint.
// @param dt Elapsed time since the previous call, in seconds.
// @return The saturated controller output, clamped to [out_min, out_max]. Returns 0 on bad args.
float pid_update(pid_controller_t *pid, float setpoint, float measurement, float dt);

// Clears the integrator and derivative history without touching the gains.
// MUST be called on every arm and every disarm: a stale integrator is the classic cause of a
// quad flipping the instant it is armed.
// @param pid Pointer to the controller.
void pid_reset(pid_controller_t *pid);

// Updates the gains in place, preserving the accumulated integrator so live tuning stays smooth.
// @param pid Pointer to the controller.
// @param kp, ki, kd New gains.
void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd);
