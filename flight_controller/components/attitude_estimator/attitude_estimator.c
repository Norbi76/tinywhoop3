// attitude_estimator.c - the complementary filter itself.
//
// WHAT THIS FILE DOES
//   One function does the real work: attitude_update(), called at 1 kHz from fc_task right after
//   the IMU read. Everything else is state management.
//
// HOW attitude_update() WORKS, in order
//   1. Copy the gyro rates into the body axes.  <-- THE UNVERIFIED AXIS MAPPING LIVES HERE
//   2. Integrate the rates to get a gyro-only roll/pitch prediction, and integrate yaw outright
//      (yaw has no correction source at all - no magnetometer on this airframe).
//   3. Gate the accelerometer on total magnitude. Outside 0.8..1.2 g it is not measuring gravity,
//      so the sample is dropped and the filter coasts on step 2 alone.
//   4. Before `initialized`: seed roll/pitch straight FROM the accelerometer rather than fusing,
//      so the filter does not have to creep in from zero over its own time constant while the
//      pilot is waiting to arm.
//   5. After `initialized`: blend prediction and measurement with alpha = tau/(tau+dt).
//
// WHY THERE IS NO LOCKING
//   All state is module-static and fc_task is the only writer. telemetry_task does call
//   attitude_get() for the status frame, and that copy is not atomic - a status frame can mix
//   fields from two consecutive ticks. Harmless for a 50 Hz display; would NOT be acceptable if a
//   control path ever read this cross-task.
//
// dt IS PASSED IN, not assumed. Loop jitter is then absorbed correctly instead of being
// integrated as an angle error.

#include "attitude_estimator.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "ATTITUDE";

// ---------------------------------------------------------------------------
// Accelerometer gating window.
// The accelerometer only tells us which way is down when the drone is not accelerating.
// During aggressive manoeuvres or prop wash the measured magnitude departs from 1 g and the
// derived angle is garbage, so we gate it out and coast on the gyro for those samples.
// ---------------------------------------------------------------------------
#define ACCEL_GATE_MIN_G 0.8f
#define ACCEL_GATE_MAX_G 1.2f

// Number of consecutive good accelerometer samples required before the filter declares itself
// initialised. At 1 kHz this is 0.5 s of the drone sitting still.
#define INIT_SAMPLES_REQUIRED 500

static attitude_state_t state;
static float filter_tau = 0.5f;
static uint32_t good_sample_count;

// Wraps an angle in degrees into [-180, 180].
static float wrap_180(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

esp_err_t attitude_init(float nominal_dt, float tau) {
    if (nominal_dt <= 0.0f || tau <= 0.0f) {
        ESP_LOGE(TAG, "attitude_init needs positive dt and tau (got dt=%.6f, tau=%.3f)", nominal_dt, tau);
        return ESP_ERR_INVALID_ARG;
    }

    memset(&state, 0, sizeof(state));
    filter_tau = tau;
    good_sample_count = 0;

    ESP_LOGI(TAG, "Attitude estimator initialised (tau=%.3f s, gate=%.2f-%.2f g)",
             filter_tau, ACCEL_GATE_MIN_G, ACCEL_GATE_MAX_G);

    return ESP_OK;
}

void attitude_update(const imu_physical_data_t *imu, float dt) {
    if (imu == NULL || dt <= 0.0f) {
        return;
    }

    // -----------------------------------------------------------------------
    // GYRO AXIS MAPPING - VERIFY ON THE BENCH BEFORE FLYING.
    //
    // This assumes the MPU6500 is mounted with its X axis pointing forward, Y axis pointing
    // right, Z axis pointing up, and that the sensor's own axes are not rotated relative to
    // the airframe. That is an assumption about how the IMU sits on YOUR PCB, and it is wrong
    // as often as it is right.
    //
    // Bench check: with the drone level and disarmed, watch the status telemetry and
    //   - roll the airframe RIGHT  -> roll_rate must go POSITIVE
    //   - pitch the NOSE UP        -> pitch_rate must go POSITIVE
    //   - yaw the NOSE RIGHT       -> yaw_rate must go POSITIVE
    // If an axis reads backwards, negate it here. If two axes are swapped, swap them here.
    // Fixing it here fixes it for the whole cascade.
    // -----------------------------------------------------------------------
    state.roll_rate  = imu->gyro_x_dps;
    state.pitch_rate = imu->gyro_y_dps;
    state.yaw_rate   = imu->gyro_z_dps;

    // --- Gyro integration (always runs) -------------------------------------
    float roll_gyro  = state.roll  + (state.roll_rate  * dt);
    float pitch_gyro = state.pitch + (state.pitch_rate * dt);

    // Yaw is pure integration. It drifts, and nothing here corrects it.
    state.yaw = wrap_180(state.yaw + (state.yaw_rate * dt));

    // --- Accelerometer gate --------------------------------------------------
    const float accel_magnitude = sqrtf((imu->acc_x_g * imu->acc_x_g) +
                                        (imu->acc_y_g * imu->acc_y_g) +
                                        (imu->acc_z_g * imu->acc_z_g));

    state.accel_valid = (accel_magnitude >= ACCEL_GATE_MIN_G && accel_magnitude <= ACCEL_GATE_MAX_G);

    if (!state.accel_valid) {
        // Gated out: coast on the gyro for this sample and do not count it towards init.
        state.roll = roll_gyro;
        state.pitch = pitch_gyro;
        return;
    }

    // Accelerometer-derived angles. imu_compute_roll_pitch() applies the level-calibration
    // offsets captured by imu_calibrate_acc() at boot, so these are already zeroed.
    float roll_accel = 0.0f, pitch_accel = 0.0f;
    imu_compute_roll_pitch(imu->acc_x_g, imu->acc_y_g, imu->acc_z_g, &roll_accel, &pitch_accel);

    if (!state.initialized) {
        // Seed straight from the accelerometer rather than letting the filter creep in from
        // zero, otherwise the first half-second of flight fights a bogus initial angle.
        state.roll = roll_accel;
        state.pitch = pitch_accel;

        good_sample_count++;
        if (good_sample_count >= INIT_SAMPLES_REQUIRED) {
            state.initialized = true;
            ESP_LOGI(TAG, "Attitude initialised: roll=%.2f pitch=%.2f", state.roll, state.pitch);
        }
        return;
    }

    // --- Complementary fusion -------------------------------------------------
    // alpha = tau / (tau + dt). With tau=0.5 s at 1 kHz that is 0.998, i.e. the gyro carries
    // the short-term response and the accelerometer only pulls the long-term bias out.
    const float alpha = filter_tau / (filter_tau + dt);

    state.roll  = (alpha * roll_gyro)  + ((1.0f - alpha) * roll_accel);
    state.pitch = (alpha * pitch_gyro) + ((1.0f - alpha) * pitch_accel);
}

void attitude_get(attitude_state_t *out) {
    if (out == NULL) {
        return;
    }
    *out = state;
}

bool attitude_is_initialized(void) {
    return state.initialized;
}

void attitude_reset(void) {
    const float saved_tau = filter_tau;
    memset(&state, 0, sizeof(state));
    filter_tau = saved_tau;
    good_sample_count = 0;
}
