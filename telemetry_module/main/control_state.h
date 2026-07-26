#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "telemetry_uart.h"

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
typedef struct {
    bool pitch_forward;
    bool pitch_back;
    bool roll_left;
    bool roll_right;
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
