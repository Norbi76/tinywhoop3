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
//   3. LOW-PASS the accelerometer, then gate it on total magnitude. Outside the window it is not
//      measuring gravity, so the sample is dropped and the filter coasts on step 2 alone.
//      The low-pass was added 2026-08-29 and is the fix for in-flight roll/pitch drift: without
//      it, prop vibration failed the gate on most samples and the filter degenerated into pure
//      gyro integration. See the long comment at ACCEL_LPF_CUTOFF_HZ.
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
// ACCELEROMETER LOW-PASS  (added 2026-08-29 - this is the fix for in-flight roll/pitch drift)
//
// THE BUG THIS FIXES
//   The gate below used to run on the RAW accelerometer magnitude at 1 kHz. Prop vibration on a
//   whoop swings the instantaneous magnitude far past +/-0.2 g, so most samples failed the gate,
//   the filter coasted on pure gyro integration for long stretches, and gyro bias integrated into
//   roll/pitch error over minutes of flight.
//
//   The 2026-08-27 flight log is unambiguous: with the motors OFF, drift was 0.0-0.3 deg/min, so
//   the filter itself is fine. Drift appeared only under power. The accelerometer was not wrong -
//   it was being thrown away.
//
// WHY A LOW-PASS RATHER THAN JUST WIDENING THE GATE
//   Vibration is high frequency; gravity is DC. They are separable in frequency, so filtering
//   REMOVES the interference instead of merely tolerating it. Widening the gate on its own would
//   have admitted the vibration into the angle calculation, trading drift for noise. Both changes
//   are here, but the filter does the work: the gate is widened only because it now sees a much
//   cleaner signal and no longer has to be defensive.
//
// WHY THE ADDED PHASE LAG DOES NOT MATTER
//   The complementary filter already weights the accelerometer at (1 - alpha) = dt/(tau+dt),
//   about 0.2% per sample at tau = 0.5 s. The accelerometer is purely a long-term levelling
//   reference here; the gyro carries every fast transient. Tens of milliseconds of lag on a
//   reference with a 0.5 s time constant is not observable in the output.
//
// TODO(bench): 15 Hz is a sensible starting point for a whoop, not a measured value. If drift
// persists, LOWER it (more attenuation, more lag) before touching the gate, and watch the accept
// ratio logged below.
// ---------------------------------------------------------------------------
#define ACCEL_LPF_CUTOFF_HZ 15.0f

// ---------------------------------------------------------------------------
// Accelerometer gating window - now applied to the FILTERED signal.
// The accelerometer only tells us which way is down when the drone is not accelerating. During
// aggressive manoeuvres the filtered magnitude genuinely departs from 1 g and the derived angle
// really is garbage, so the gate still earns its place: it now rejects sustained real
// acceleration rather than prop buzz.
//
// Widened from 0.8-1.2 g. On the filtered signal the resting magnitude sits very close to 1 g, so
// this is less loose than it looks - it is sized to survive throttle transients without dropping
// the levelling reference for seconds at a time.
// ---------------------------------------------------------------------------
#define ACCEL_GATE_MIN_G 0.7f
#define ACCEL_GATE_MAX_G 1.3f

// ---------------------------------------------------------------------------
// Periodic health log: what fraction of the last N samples the gate ACCEPTED.
//
// That single number is the whole diagnosis. A healthy fused filter accepts most samples; the
// failure mode that caused the drift is a low accept ratio under power. It was previously
// invisible, because accel_valid is not carried in the 50 Hz status frame - neither the dashboard
// nor the flight logs could show it. Watch it over USB serial with the props spinning; if it is
// not comfortably high, lower ACCEL_LPF_CUTOFF_HZ.
//
// 1000 samples at 1 kHz is one line per second.
// ---------------------------------------------------------------------------
#define ATTITUDE_HEALTH_LOG_ENABLED 1
#define ATTITUDE_HEALTH_LOG_SAMPLES 1000

// Number of consecutive good accelerometer samples required before the filter declares itself
// initialised. At 1 kHz this is 0.5 s of the drone sitting still.
#define INIT_SAMPLES_REQUIRED 500

static attitude_state_t state;
static float filter_tau = 0.5f;
static uint32_t good_sample_count;

// Low-passed accelerometer, in g. Seeded from the first sample rather than from zero so the
// filter does not have to climb from 0 g through the gate's lower bound before anything is
// accepted - that would have delayed `initialized` by roughly the filter's own time constant.
static float accel_lpf_x, accel_lpf_y, accel_lpf_z;
static bool accel_lpf_primed;

// Rolling accept-ratio counters for the health log.
static uint32_t health_total, health_accepted;

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
    accel_lpf_primed = false;
    health_total = 0;
    health_accepted = 0;

    ESP_LOGI(TAG, "Attitude estimator initialised (tau=%.3f s, accel LPF %.1f Hz, gate=%.2f-%.2f g)",
             filter_tau, ACCEL_LPF_CUTOFF_HZ, ACCEL_GATE_MIN_G, ACCEL_GATE_MAX_G);

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

    // --- Accelerometer low-pass ----------------------------------------------
    // First-order RC, recomputed from the measured dt each call so loop jitter does not change
    // the effective cutoff. Filtering the three components separately (rather than the magnitude)
    // is what matters: the ANGLE is derived from the component ratios, so the components are what
    // must be clean.
    if (!accel_lpf_primed) {
        accel_lpf_x = imu->acc_x_g;
        accel_lpf_y = imu->acc_y_g;
        accel_lpf_z = imu->acc_z_g;
        accel_lpf_primed = true;
    } else {
        const float rc = 1.0f / (2.0f * (float)M_PI * ACCEL_LPF_CUTOFF_HZ);
        const float lpf_alpha = dt / (rc + dt);
        accel_lpf_x += lpf_alpha * (imu->acc_x_g - accel_lpf_x);
        accel_lpf_y += lpf_alpha * (imu->acc_y_g - accel_lpf_y);
        accel_lpf_z += lpf_alpha * (imu->acc_z_g - accel_lpf_z);
    }

    // --- Accelerometer gate --------------------------------------------------
    // Gate on the FILTERED magnitude. On the raw signal this rejected most samples in powered
    // flight and the filter degenerated into gyro-only integration - see the comment at
    // ACCEL_LPF_CUTOFF_HZ.
    const float accel_magnitude = sqrtf((accel_lpf_x * accel_lpf_x) +
                                        (accel_lpf_y * accel_lpf_y) +
                                        (accel_lpf_z * accel_lpf_z));

    state.accel_valid = (accel_magnitude >= ACCEL_GATE_MIN_G && accel_magnitude <= ACCEL_GATE_MAX_G);

#if ATTITUDE_HEALTH_LOG_ENABLED
    health_total++;
    if (state.accel_valid) health_accepted++;
    if (health_total >= ATTITUDE_HEALTH_LOG_SAMPLES) {
        ESP_LOGI(TAG, "accel accept %lu%% (|a| %.3f g) roll %+.2f pitch %+.2f",
                 (unsigned long)((health_accepted * 100u) / health_total),
                 (double)accel_magnitude, (double)state.roll, (double)state.pitch);
        health_total = 0;
        health_accepted = 0;
    }
#endif

    if (!state.accel_valid) {
        // Gated out: coast on the gyro for this sample and do not count it towards init.
        state.roll = roll_gyro;
        state.pitch = pitch_gyro;
        return;
    }

    // Accelerometer-derived angles, from the FILTERED components. imu_compute_roll_pitch()
    // applies the level-calibration offsets captured by imu_calibrate_acc() at boot, so these are
    // already zeroed. Offset subtraction is linear, so filtering before it is equivalent to
    // filtering after - the order here is not load-bearing.
    float roll_accel = 0.0f, pitch_accel = 0.0f;
    imu_compute_roll_pitch(accel_lpf_x, accel_lpf_y, accel_lpf_z, &roll_accel, &pitch_accel);

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

    // Drop the low-pass state too, so a reset re-seeds from the next real sample instead of
    // carrying the previous attitude's accelerometer history across the discontinuity.
    accel_lpf_primed = false;
    health_total = 0;
    health_accepted = 0;
}
