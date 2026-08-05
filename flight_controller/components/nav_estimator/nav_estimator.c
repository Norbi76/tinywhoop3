#include "nav_estimator.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "NAV";

#define DEG_TO_RAD (float)(M_PI / 180.0)

// ---------------------------------------------------------------------------
// TODO(bench): FLOW_COUNTS_PER_RAD - MEASURE THIS.
// The PMW3901 reports motion in its own internal counts. This constant converts counts to
// radians of angular displacement and is the single scale factor that sets how fast the drone
// thinks it is moving. Everything about velocity hold depends on getting it right.
//
// How to measure: mount the drone on a slide or a ruler at a known fixed height (say 300 mm),
// translate it a known distance (say 200 mm) at a steady speed, and log the accumulated
// delta counts. Then:
//     angle_moved_rad   = atan2(0.200, 0.300)
//     FLOW_COUNTS_PER_RAD = accumulated_counts / angle_moved_rad
// Repeat on both axes. The published figure for the PMW3901's ~42 degree field of view over
// 30x30 pixels lands somewhere near 500, but that is a starting point for a sanity check,
// not a substitute for measuring your own unit.
// ---------------------------------------------------------------------------
#define FLOW_COUNTS_PER_RAD 500.0f

// Altitude is unusable past this tilt: the cos() correction stops being a good model and the
// ToF's narrow cone starts ranging against a wall or furniture rather than the floor.
#define MAX_TILT_FOR_ALTITUDE_DEG 40.0f

// ToF range limits, in metres. Short distance mode tops out around 1.3 m; readings at the
// very bottom of the scale are usually the drone's own landing gear or the ground effect.
#define MIN_VALID_ALTITUDE_M 0.03f
#define MAX_VALID_ALTITUDE_M 1.30f

// If no good range or flow arrives within this long, drop the corresponding validity flag.
// Sized to tolerate a couple of dropped samples without flapping the flag.
#define RANGE_TIMEOUT_US 200000  // 200 ms
#define FLOW_TIMEOUT_US  200000

// Low-pass smoothing factors (0..1, higher = less filtering).
// Climb rate is a differentiated quantity so it needs heavy smoothing; velocity less so.
#define CLIMB_RATE_ALPHA 0.20f
#define VELOCITY_ALPHA   0.35f

static nav_state_t state;
static float previous_altitude;
static bool has_previous_altitude;
static int64_t last_good_range_us;
static int64_t last_good_flow_us;

static float clampf(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

esp_err_t nav_estimator_init(float nominal_range_dt) {
    if (nominal_range_dt <= 0.0f) {
        ESP_LOGE(TAG, "nav_estimator_init needs a positive dt (got %.6f)", nominal_range_dt);
        return ESP_ERR_INVALID_ARG;
    }

    nav_estimator_reset();

    ESP_LOGI(TAG, "Nav estimator initialised (flow scale %.1f counts/rad, altitude %.2f-%.2f m)",
             (double)FLOW_COUNTS_PER_RAD, (double)MIN_VALID_ALTITUDE_M, (double)MAX_VALID_ALTITUDE_M);

    return ESP_OK;
}

void nav_estimator_update_range(const vl53l1x_result_t *range, float roll_deg, float pitch_deg, float dt) {
    const int64_t now_us = esp_timer_get_time();

    // --- Gate 1: did we even get a reading, and did the sensor like it? ------
    if (range == NULL || !range->valid || range->range_status != 0) {
        if ((now_us - last_good_range_us) > RANGE_TIMEOUT_US) {
            state.altitude_valid = false;
            state.velocity_valid = false;   // velocity scaling needs altitude, so it goes too
            has_previous_altitude = false;
        }
        return;
    }

    // --- Gate 2: is the drone tilted so far that the beam misses the floor? --
    const float tilt_deg = fabsf(roll_deg) > fabsf(pitch_deg) ? fabsf(roll_deg) : fabsf(pitch_deg);
    if (tilt_deg > MAX_TILT_FOR_ALTITUDE_DEG) {
        if ((now_us - last_good_range_us) > RANGE_TIMEOUT_US) {
            state.altitude_valid = false;
            state.velocity_valid = false;
            has_previous_altitude = false;
        }
        return;
    }

    // --- Tilt compensation --------------------------------------------------
    // The ToF measures slant range along its own boresight. With the sensor pointing straight
    // down through the airframe, the vertical component is range * cos(roll) * cos(pitch).
    const float slant_range_m = range->distance_mm / 1000.0f;
    const float altitude_m = slant_range_m * cosf(roll_deg * DEG_TO_RAD) * cosf(pitch_deg * DEG_TO_RAD);

    // --- Gate 3: plausible height? ------------------------------------------
    if (altitude_m < MIN_VALID_ALTITUDE_M || altitude_m > MAX_VALID_ALTITUDE_M) {
        if ((now_us - last_good_range_us) > RANGE_TIMEOUT_US) {
            state.altitude_valid = false;
            state.velocity_valid = false;
            has_previous_altitude = false;
        }
        return;
    }

    // --- Climb rate ----------------------------------------------------------
    if (has_previous_altitude && dt > 0.0f) {
        const float raw_climb_rate = (altitude_m - previous_altitude) / dt;
        state.climb_rate += CLIMB_RATE_ALPHA * (raw_climb_rate - state.climb_rate);
    } else {
        state.climb_rate = 0.0f;
    }

    previous_altitude = altitude_m;
    has_previous_altitude = true;

    state.altitude = altitude_m;
    state.altitude_valid = true;
    last_good_range_us = now_us;
}

void nav_estimator_update_flow(const pmw3901_motion_t *flow, float roll_rate_dps, float pitch_rate_dps, float dt) {
    const int64_t now_us = esp_timer_get_time();

    // --- Gate 1: usable flow reading over a surface with enough texture? -----
    if (flow == NULL || !flow->valid) {
        if ((now_us - last_good_flow_us) > FLOW_TIMEOUT_US) {
            state.velocity_valid = false;
        }
        return;
    }

    // --- Gate 2: altitude is mandatory --------------------------------------
    // Flow is an ANGULAR rate. The same angular rate means 10 cm/s at 20 cm altitude and
    // 50 cm/s at 1 m. Without a trusted altitude the velocity number is meaningless, so we
    // refuse to produce one rather than emit a plausible-looking wrong value.
    if (!state.altitude_valid || dt <= 0.0f) {
        state.velocity_valid = false;
        return;
    }

    // --- Counts to radians ---------------------------------------------------
    const float flow_x_rad = flow->delta_x / FLOW_COUNTS_PER_RAD;
    const float flow_y_rad = flow->delta_y / FLOW_COUNTS_PER_RAD;

    // --- Gyro compensation ---------------------------------------------------
    // The sensor cannot tell translation from rotation: pitching the nose down scrolls the
    // image exactly like flying forward does. Subtract the rotation the gyro measured over the
    // same interval, and what is left is genuine translation.
    //
    // AXIS PAIRING AND SIGNS NEED BENCH VERIFICATION.
    // This assumes the flow sensor's X axis aligns with the body X (forward) axis and that a
    // positive pitch rate produces positive flow_x. Both are assumptions about how the sensor
    // is physically rotated on your PCB.
    //
    // Bench check, drone held at a fixed height over a textured surface:
    //   - slide the drone FORWARD without tilting -> velocity_x must go POSITIVE
    //   - slide the drone RIGHT without tilting   -> velocity_y must go POSITIVE
    //   - PITCH the drone in place without translating -> velocity_x must stay near ZERO
    //     (if it swings, the gyro compensation sign below is wrong)
    //   - ROLL the drone in place without translating  -> velocity_y must stay near ZERO
    const float rotation_x_rad = (pitch_rate_dps * DEG_TO_RAD) * dt;
    const float rotation_y_rad = (roll_rate_dps * DEG_TO_RAD) * dt;

    const float translation_x_rad = flow_x_rad - rotation_x_rad;
    const float translation_y_rad = flow_y_rad - rotation_y_rad;

    // --- Angular rate to linear velocity -------------------------------------
    // velocity = (angular displacement / dt) * height
    const float raw_velocity_x = (translation_x_rad / dt) * state.altitude;
    const float raw_velocity_y = (translation_y_rad / dt) * state.altitude;

    // Reject obvious nonsense before it reaches the filter. An indoor whoop is not doing 5 m/s.
    const float velocity_limit = 5.0f;
    if (fabsf(raw_velocity_x) > velocity_limit || fabsf(raw_velocity_y) > velocity_limit) {
        return;
    }

    state.velocity_x += VELOCITY_ALPHA * (raw_velocity_x - state.velocity_x);
    state.velocity_y += VELOCITY_ALPHA * (raw_velocity_y - state.velocity_y);

    state.velocity_x = clampf(state.velocity_x, -velocity_limit, velocity_limit);
    state.velocity_y = clampf(state.velocity_y, -velocity_limit, velocity_limit);

    state.velocity_valid = true;
    last_good_flow_us = now_us;
}

void nav_estimator_get(nav_state_t *out) {
    if (out == NULL) {
        return;
    }
    *out = state;
}

void nav_estimator_reset(void) {
    memset(&state, 0, sizeof(state));
    previous_altitude = 0.0f;
    has_previous_altitude = false;
    last_good_range_us = 0;
    last_good_flow_us = 0;
}
