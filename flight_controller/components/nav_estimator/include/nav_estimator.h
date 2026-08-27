#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "vl53l1x_driver.h"
#include "pmw3901_driver.h"

// nav_estimator.h - "where am I": altitude and body-frame velocity for the outer control loop.
//
// WHAT THIS COMPONENT IS FOR
//   The only source of position information on this aircraft - there is no GPS, no barometer and
//   no magnetometer. It takes raw millimetres from the ToF and raw counts from the optical flow
//   sensor and produces altitude, climb rate, and forward/right velocity.
//
// HOW IT DOES IT
//   Altitude: slant range * cos(roll) * cos(pitch), gated on sensor status, tilt and plausible
//   height, with the climb rate differentiated from it and heavily low-passed.
//   Velocity:  flow counts -> radians -> subtract the rotation the gyro saw over the same
//   interval (the sensor cannot tell "flying forward" from "pitching down") -> multiply by
//   altitude to turn an angular rate into m/s.
//
// THE VALIDITY FLAGS ARE THE MOST IMPORTANT PART OF THIS HEADER.
//   Read the comment on the struct below. Both flags go false during NORMAL flight with HEALTHY
//   hardware, and callers that ignore them will fly on numbers that are simply wrong.
//
// Two scale/sign conventions here are unmeasured guesses - see README.md and nav_estimator.c.
//
// Navigation estimator: turns raw ToF range and optical flow counts into altitude and
// body-frame velocity that the outer control loops can use.
//
// Both outputs carry a validity flag, and BOTH CAN GO FALSE AT ANY TIME - over a featureless
// floor, at high tilt angles, above the ToF's range ceiling, or simply because the hardware is
// not fitted. flight_control checks these flags on every outer-loop iteration and disengages
// the affected loop when they are false. Nothing downstream may assume these values are good.
typedef struct {
    float altitude;      // metres above the surface directly below, tilt compensated
    float climb_rate;    // metres/second, + = climbing
    float velocity_x;    // body-frame FORWARD velocity, m/s   (SIGN NEEDS BENCH VERIFICATION)
    float velocity_y;    // body-frame RIGHT velocity, m/s      (SIGN NEEDS BENCH VERIFICATION)

    bool altitude_valid; // ToF is returning good ranges and the drone is not tilted too far
    bool velocity_valid; // flow surface quality is adequate AND altitude_valid is true
} nav_state_t;

// Initialises the estimator and clears its state.
// @param nominal_range_dt Expected period between range updates in seconds (1/33 s).
// @return ESP_OK on success, ESP_ERR_INVALID_ARG for a non-positive dt.
esp_err_t nav_estimator_init(float nominal_range_dt);

// Folds in one ToF measurement and updates altitude and climb rate.
// @param range Result from vl53l1x_read(), or NULL to signal a failed/missing read.
// @param roll_deg Current roll angle in degrees, for tilt compensation.
// @param pitch_deg Current pitch angle in degrees.
// @param dt Seconds since the previous range update.
void nav_estimator_update_range(const vl53l1x_result_t *range, float roll_deg, float pitch_deg, float dt);

// Folds in one optical flow measurement and updates body-frame velocity.
// Requires a valid altitude: flow measures ANGULAR motion, and converting that to metres per
// second needs to know how far away the surface is.
// @param flow Result from pmw3901_read_motion(), or NULL to signal a failed/missing read.
// @param roll_rate_dps Body roll rate in degrees/second, for rotation compensation.
// @param pitch_rate_dps Body pitch rate in degrees/second.
// @param dt Seconds since the previous flow update.
void nav_estimator_update_flow(const pmw3901_motion_t *flow, float roll_rate_dps, float pitch_rate_dps, float dt);

// Copies the current navigation state out.
// @param out Destination, must not be NULL.
void nav_estimator_get(nav_state_t *out);

// Clears all state and drops both validity flags.
void nav_estimator_reset(void);
