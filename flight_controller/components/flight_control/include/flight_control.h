#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "attitude_estimator.h"
#include "nav_estimator.h"
#include "telemetry_uart.h"

// flight_control.h - the orchestrator. Everything else on this board feeds it or is driven by it.
//
// WHAT THIS COMPONENT IS FOR
//   Owns the eight PID instances, decides when the motors are allowed to run at all, converts
//   three axis commands into four motor thrusts, and disarms the aircraft when the pilot's link
//   goes away. flight_control_update() is the only thing that writes the motors in normal
//   operation.
//
// HOW IT DOES ITS JOB
//   Three nested loops (below) clocked by DIVIDER COUNTERS off one 1 kHz tick rather than by
//   separate tasks - so they can never drift in phase against each other, and the whole cascade
//   is one call stack with no synchronisation inside it. Around that sit the arming state
//   machine (kill / link watchdog / four-condition arming gate) and the X-quad mixer.
//
// THE SAFETY INVARIANT
//   Every path that does not EXPLICITLY authorise the motors ends at motor_all_stop(). That is
//   structured as a single early return in flight_control_update() rather than as conditionals
//   scattered through the cascade, so "disarmed means stopped" is auditable by reading one
//   function.
//
// The mixer signs and two constants in flight_control.c are unverified on hardware and the drone
// can flip on the first flight if they are wrong. See README.md before flying.
//
// Flight control orchestrator: the cascade, the mixer, the arming state machine and the
// control-link watchdog.
//
// CASCADE STRUCTURE - three nested loops, all clocked off the 1 kHz fc_task tick using
// divider counters so every loop stays phase-locked to the others:
//
//   OUTER   50 Hz  (every 20 ticks)  altitude -> throttle,  velocity -> angle setpoints
//   MID    250 Hz  (every 4 ticks)   angle    -> rate setpoints
//   INNER 1000 Hz  (every tick)      rate     -> mixer -> motors
//
// The outer loop is the only one that depends on nav_estimator, and it disengages by itself
// the moment altitude_valid or velocity_valid goes false. The mid and inner loops only need
// the IMU, so the drone stays flyable with no ToF and no optical flow fitted at all.

// Arming state.
typedef enum {
    FLIGHT_STATE_DISARMED = 0, // Motors forced to zero. The only state you can power up in.
    FLIGHT_STATE_ARMED = 1,    // Motors live.
    FLIGHT_STATE_KILLED = 2,   // Latched emergency stop. Requires an explicit disarm to leave.
} flight_state_t;

// Initialises all 8 PID instances with their default gains and puts the controller in
// FLIGHT_STATE_DISARMED.
// @return ESP_OK on success, or the first PID init error.
esp_err_t flight_control_init(void);

// Hands a freshly received control frame to the controller and resets the link watchdog.
// Safe to call from a different task and a different core than flight_control_update();
// the copy is done inside a spinlock-protected critical section.
// @param control The decoded control payload. NULL is ignored.
void flight_control_set_control_input(const telemetry_control_payload_t *control);

// Runs one iteration of the cascade and writes the motors. Call at exactly 1 kHz from fc_task.
// Handles arming, the link watchdog, the loop dividers, the mixer, and motor output. On any
// path that ends disarmed this calls motor_all_stop() before returning.
// @param attitude Current attitude estimate.
// @param nav Current navigation state. Pass NULL if nav is not available at all.
// @param dt Seconds since the previous call.
void flight_control_update(const attitude_state_t *attitude, const nav_state_t *nav, float dt);

// Fills in a status frame for transmission back to the telemetry module.
// @param out Destination, must not be NULL.
void flight_control_get_status(telemetry_status_payload_t *out);

// True while the motors are live.
bool flight_control_is_armed(void);

// Returns the current arming state.
flight_state_t flight_control_get_state(void);

// --- Live gain tuning -------------------------------------------------------
// All of these are safe to call while flying; the affected PID keeps its accumulated
// integrator so the change does not produce a step in the output.

// Sets the gains of one loop by id.
// @param loop_id Which of the 8 PID instances, see telemetry_loop_id_t.
// @param kp, ki, kd New gains.
// @return ESP_OK on success, ESP_ERR_INVALID_ARG for an out-of-range loop id.
esp_err_t flight_control_set_gains(telemetry_loop_id_t loop_id, float kp, float ki, float kd);

// Sets both roll and pitch rate loops at once (they are almost always tuned together).
void flight_control_set_rate_gains(float kp, float ki, float kd);

// Sets both roll and pitch angle loops at once.
void flight_control_set_angle_gains(float kp, float ki, float kd);

// Sets both body velocity loops at once.
void flight_control_set_velocity_gains(float kp, float ki, float kd);

// Sets the altitude loop.
void flight_control_set_altitude_gains(float kp, float ki, float kd);
