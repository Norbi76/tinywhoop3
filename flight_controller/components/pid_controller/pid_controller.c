// pid_controller.c - the control law.
//
// WHAT THIS FILE DOES
//   pid_init()       validates arguments, zeroes the struct, and derives the derivative filter
//                    coefficient d_alpha from the requested cutoff and the nominal loop period.
//   pid_update()     one iteration: P, then filtered derivative-on-measurement, then integral
//                    with conditional-integration anti-windup, then clamp. Also records the
//                    telemetry fields pid_registry publishes.
//   pid_reset()      clears integrator + derivative history, keeps gains.  (arm/disarm path)
//   pid_set_gains()  changes gains, keeps the integrator.                  (live tuning path)
//
// READING pid_update()
//   The order is not arbitrary. P and D are computed FIRST because the anti-windup test needs
//   the full candidate output - it has to know what the sum would be before deciding whether
//   committing this integration step would push further into saturation. Each of the four
//   non-obvious choices is explained at the point where it happens.
//
// NO STATE LIVES IN THIS FILE. Every instance is entirely contained in the caller's struct, so
// there is nothing to lock and nothing shared between the eight instances.

#include "pid_controller.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "PID";

static float clampf(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

esp_err_t pid_init(pid_controller_t *pid,
                   float kp, float ki, float kd,
                   float out_min, float out_max,
                   float integrator_limit,
                   float d_cutoff_hz,
                   float nominal_dt) {
    if (pid == NULL) {
        ESP_LOGE(TAG, "pid_init got a NULL controller");
        return ESP_ERR_INVALID_ARG;
    }
    if (nominal_dt <= 0.0f || d_cutoff_hz <= 0.0f) {
        ESP_LOGE(TAG, "pid_init needs a positive dt and cutoff (got dt=%.6f, fc=%.2f)", nominal_dt, d_cutoff_hz);
        return ESP_ERR_INVALID_ARG;
    }
    if (out_min >= out_max) {
        ESP_LOGE(TAG, "pid_init needs out_min < out_max (got %.3f, %.3f)", out_min, out_max);
        return ESP_ERR_INVALID_ARG;
    }

    memset(pid, 0, sizeof(*pid));

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integrator_limit = fabsf(integrator_limit);

    // Single-pole RC low-pass on the derivative:
    //   RC    = 1 / (2*pi*fc)
    //   alpha = dt / (RC + dt)
    // alpha near 1 means almost no filtering, alpha near 0 means very heavy filtering.
    const float rc = 1.0f / (2.0f * (float)M_PI * d_cutoff_hz);
    pid->d_alpha = nominal_dt / (rc + nominal_dt);
    pid->d_cutoff_hz = d_cutoff_hz;
    pid->nominal_dt = nominal_dt;

    pid->first_update = true;

    return ESP_OK;
}

float pid_update(pid_controller_t *pid, float setpoint, float measurement, float dt) {
    if (pid == NULL || dt <= 0.0f) {
        return 0.0f;
    }

    const float error = setpoint - measurement;

    // --- Proportional ---------------------------------------------------
    const float p_term = pid->kp * error;

    // --- Derivative on MEASUREMENT --------------------------------------
    // d(error)/dt == d(setpoint)/dt - d(measurement)/dt. We deliberately drop the setpoint
    // half so that a step in the setpoint does not produce a derivative spike ("setpoint kick").
    // The sign flip is what is left: -d(measurement)/dt.
    float d_raw = 0.0f;
    if (pid->first_update) {
        // No previous sample yet, so there is no meaningful rate of change. Seed and emit zero.
        pid->first_update = false;
    } else {
        d_raw = -(measurement - pid->prev_measurement) / dt;
    }
    pid->prev_measurement = measurement;

    pid->d_filtered += pid->d_alpha * (d_raw - pid->d_filtered);
    const float d_term = pid->kd * pid->d_filtered;

    // --- Integral with conditional integration --------------------------
    // Work out what the output would be if we did integrate this sample, then only commit the
    // new integrator value if it does not drive us further into saturation.
    const float candidate_integrator = pid->integrator + (pid->ki * error * dt);
    const float candidate_output = p_term + candidate_integrator + d_term;

    bool freeze_integrator = false;
    if (candidate_output > pid->out_max && error > 0.0f) {
        // Already commanding more than we can deliver and the error asks for even more.
        freeze_integrator = true;
    } else if (candidate_output < pid->out_min && error < 0.0f) {
        freeze_integrator = true;
    }

    bool hit_integrator_limit = false;
    if (!freeze_integrator) {
        const float clamped_integrator =
            clampf(candidate_integrator, -pid->integrator_limit, pid->integrator_limit);
        hit_integrator_limit = (clamped_integrator != candidate_integrator);
        pid->integrator = clamped_integrator;
    }

    // --- Output ---------------------------------------------------------
    const float raw_output = p_term + pid->integrator + d_term;
    const float output = clampf(raw_output, pid->out_min, pid->out_max);

    pid->last_p = p_term;
    pid->last_i = pid->integrator;
    pid->last_d = d_term;

    // Telemetry only, see pid_controller_t. Recording the two saturation conditions here is the
    // only place they are observable - both are computed above and would otherwise be discarded,
    // and "why is this loop not responding" is almost always one of them.
    pid->last_setpoint = setpoint;
    pid->last_measurement = measurement;
    pid->last_output = output;
    pid->out_saturated = (output != raw_output);
    pid->integrator_clamped = freeze_integrator || hit_integrator_limit;

    return output;
}

void pid_reset(pid_controller_t *pid) {
    if (pid == NULL) {
        return;
    }

    pid->integrator = 0.0f;
    pid->d_filtered = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->first_update = true;
    pid->last_p = 0.0f;
    pid->last_i = 0.0f;
    pid->last_d = 0.0f;
    pid->last_setpoint = 0.0f;
    pid->last_measurement = 0.0f;
    pid->last_output = 0.0f;
    pid->out_saturated = false;
    pid->integrator_clamped = false;
}

void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd) {
    if (pid == NULL) {
        return;
    }

    // The integrator is left alone on purpose - see the comment on pid_controller_t.integrator.
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
