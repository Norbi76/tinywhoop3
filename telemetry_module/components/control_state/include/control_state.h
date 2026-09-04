#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "telemetry_uart.h"

// control_state.h - the pilot's intent, held in one place.
//
// WHAT THIS COMPONENT IS FOR
//   The dashboard has BUTTONS, not joysticks. Something has to synthesise a continuous setpoint
//   from a discrete "is this key down right now", and this is that something. It is also where
//   the last status frame from the flight controller is parked, so the HTTP layer can serve the
//   dashboard without ever touching the UART.
//
// HOW IT DOES ITS JOB
//   Ramp-on-hold, decay-on-release integration (decay ~2x ramp, so releasing always settles the
//   drone faster than pressing moved it), driven at a FIXED 50 Hz from the UART TX task rather
//   than from the HTTP handler - so the ramp rates do not silently change when the browser
//   stutters. Throttle is the deliberate exception: a persistent trim with no decay branch.
//
//   ROLL AND PITCH ADDITIONALLY STEP on the edge of a press, before the ramp, so the FWD / BACK /
//   LEFT / RIGHT buttons produce a visible response on the first frame instead of creeping up
//   from zero. Yaw rate and throttle stay pure ramps - a step there would be a step in rate.
//
// TWO WATCHDOGS LIVE HERE, and the second one is the non-obvious one:
//   500 ms without a POST -> release directional inputs, KEEP the throttle (level out, hold height)
//     3 s without a POST -> drop the arm request outright
//   The 3 s backstop exists because the UART link stays perfectly healthy when the BROWSER dies -
//   the flight controller's own link watchdog would never fire, and the drone would hover forever.
//
// If you change any rate, limit or timeout here, mirror it in tools/mock_server.py.
//
// Turns the dashboard's held-button state into flight setpoints.
//
// The dashboard has no joysticks - it has buttons you press and hold. That means the setpoint
// has to be synthesised here: while a button is held the corresponding axis RAMPS towards its
// limit, and when it is released the axis DECAYS back to neutral. Decay is faster than ramp,
// so letting go always settles the drone faster than pressing made it move.
//
// Throttle is the exception. It is a persistent TRIM, not a stick: holding "up" raises it and
// releasing leaves it exactly where it was. A decaying throttle would mean the drone sinks the
// moment you stop pressing, which is unflyable with a button interface.

// Which directional buttons are currently held. Sent by the dashboard 20 times a second.
//
// NOTE (2026-08-29): the four roll/pitch fields are NO LONGER USED. Roll and pitch now come from
// the dashboard's analogue joystick via control_state_set_stick(). The fields are kept so the wire
// format and the HTTP handler do not have to change, and so a keyboard or button fallback could be
// reintroduced without touching the protocol - but control_state_update() ignores them. Yaw and
// throttle are still button-driven, and deliberately so: see the joystick comment in the .c file.
typedef struct {
    bool pitch_forward;   // IGNORED - joystick owns pitch
    bool pitch_back;      // IGNORED
    bool roll_left;       // IGNORED - joystick owns roll
    bool roll_right;      // IGNORED
    bool yaw_left;
    bool yaw_right;
    bool throttle_up;
    bool throttle_down;
} control_buttons_t;

// Initialises the module. All axes neutral, throttle trim zero, disarmed.
// @return ESP_OK on success, ESP_FAIL if the internal mutex could not be created.
esp_err_t control_state_init(void);

// Records the latest held-button state and resets the browser watchdog.
// Called from the /api/input HTTP handler.
// @param buttons The button state. NULL is ignored.
void control_state_set_buttons(const control_buttons_t *buttons);

// Records the analogue joystick position that drives ROLL and PITCH, and resets the browser
// watchdog. Called from the /api/input HTTP handler alongside control_state_set_buttons().
//
// Both axes are normalised stick displacement in [-1, +1] and are clamped to the unit DISC, not
// the unit square: a diagonal push must not command 1.41x the tilt of a straight one.
//
//   x  +1 = full right   (roll right, right wing down)
//   y  +1 = full forward (pitch forward, nose down -> the drone moves forward)
//
// The expo curve, the degree limit and the slew limit are all applied in control_state_update(),
// not here - the dashboard sends raw displacement and the firmware owns the feel, so every client
// gets identical behaviour and there is one place to tune.
//
// @param x Roll displacement, [-1, +1]. Non-finite values are treated as 0.
// @param y Pitch displacement, [-1, +1]. Non-finite values are treated as 0.
void control_state_set_stick(float x, float y);

// Sets the arm request. The flight controller applies its own arming gate on top of this.
void control_state_set_arm(bool armed);

// Sets the latching kill request.
void control_state_set_kill(bool kill);

// Sets the hold (altitude/position) toggle.
void control_state_set_hold(bool hold);

// Sets the requested flight mode (telemetry_flight_mode_t).
void control_state_set_mode(uint8_t mode);

// Flags that a photo was requested, so the next outgoing frame carries the capture bit.
void control_state_flag_capture(void);

// Advances the ramp/decay integration by dt and applies the browser watchdog.
// Called at a fixed 50 Hz from the UART TX task, NOT from the HTTP handler - the setpoints
// must keep evolving at a known rate whether or not the browser is posting.
// @param dt Seconds since the previous call.
void control_state_update(float dt);

// Copies the current control frame out, ready to be sent to the flight controller.
// @param out Destination, must not be NULL.
void control_state_get_frame(telemetry_control_payload_t *out);

// Current throttle trim, 0.0-1.0. Read back by the dashboard so the ALT+/ALT- buttons can show
// the pilot what they've actually asked for, not just what they last pressed.
float control_state_get_throttle_trim(void);

// Stores the most recent status frame received from the flight controller.
// @param status The decoded status payload. NULL is ignored.
void control_state_store_status(const telemetry_status_payload_t *status);

// Copies the last known flight controller status out for the dashboard to read back.
// @param out Destination, must not be NULL.
// @return true if a status frame has ever been received, false if the drone has never replied.
bool control_state_get_status(telemetry_status_payload_t *out);

// Milliseconds since the last status frame arrived, or -1 if none ever has.
int32_t control_state_status_age_ms(void);

// True if the browser has posted within the input timeout.
bool control_state_browser_connected(void);
