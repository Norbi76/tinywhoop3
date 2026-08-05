#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "imu_driver.h"

// Complementary-filter attitude estimator.
//
// Roll and pitch are fused from the accelerometer (absolute but noisy, and wrong under
// acceleration) and the gyroscope (smooth but drifts). Yaw is gyro integration ONLY - there is
// no magnetometer on this airframe, so yaw drift is unbounded and yaw is flown as a rate,
// never as an angle. Do not build anything that assumes absolute heading.
typedef struct {
    float roll;        // degrees, + = right side down    (SIGN NEEDS BENCH VERIFICATION)
    float pitch;       // degrees, + = nose up            (SIGN NEEDS BENCH VERIFICATION)
    float yaw;         // degrees, free-running, wrapped to [-180, 180]

    float roll_rate;   // degrees/second, body X axis     (AXIS MAPPING NEEDS BENCH VERIFICATION)
    float pitch_rate;  // degrees/second, body Y axis     (AXIS MAPPING NEEDS BENCH VERIFICATION)
    float yaw_rate;    // degrees/second, body Z axis     (AXIS MAPPING NEEDS BENCH VERIFICATION)

    bool accel_valid;  // false when the accelerometer was gated out on the last update
    bool initialized;  // true once the filter has settled; an arming precondition
} attitude_state_t;

// Initialises the estimator and clears its state.
// @param nominal_dt Expected update period in seconds (1.0/1000.0 for the 1 kHz loop).
// @param tau Complementary filter time constant in seconds. Larger = trusts the gyro more and
//            the accelerometer less. ~0.5 s is a reasonable starting point for a small quad.
// @return ESP_OK on success, ESP_ERR_INVALID_ARG for a non-positive dt or tau.
esp_err_t attitude_init(float nominal_dt, float tau);

// Runs one filter iteration. Call at a steady rate from fc_task.
// @param imu Physical IMU data (g and degrees/second) from imu_convert_raw_to_physical().
// @param dt Elapsed time since the previous call, in seconds.
void attitude_update(const imu_physical_data_t *imu, float dt);

// Copies the current attitude estimate out.
// @param out Destination, must not be NULL.
void attitude_get(attitude_state_t *out);

// True once the filter has seen enough good accelerometer samples to trust its roll/pitch.
// flight_control refuses to arm until this returns true.
bool attitude_is_initialized(void);

// Clears the filter state and forces re-initialisation.
void attitude_reset(void);
