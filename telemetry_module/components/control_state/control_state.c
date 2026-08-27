// control_state.c - setpoint synthesis, the browser watchdogs, and the status cache.
//
// WHAT THIS FILE DOES
//   Input side  (HTTP handlers):  set_buttons / set_arm / set_kill / set_hold / set_mode /
//                                 flag_capture - these only RECORD intent and pet the watchdog.
//   Output side (UART TX task):   control_state_update() integrates at a fixed 50 Hz, then
//                                 control_state_get_frame() packages the result for the wire.
//   Status side (UART RX task):   store_status / get_status / status_age_ms.
//
// HOW THE INTEGRATION WORKS
//   ramp_towards() is the primitive; update_axis() layers the hold/release policy on top of it
//   and is shared by roll, pitch and yaw. Throttle deliberately does NOT go through update_axis()
//   - it has no decay branch at all, which is what makes button-driven altitude possible.
//
// WHY TWO MUTEXES
//   state_mutex and status_mutex guard disjoint data touched by different tasks on different
//   clocks. The HTTP handler serving /api/status must not contend with the UART TX task
//   integrating setpoints.
//
// EVERY MUTEX TAKE HAS A 5 ms TIMEOUT, never portMAX_DELAY, and every failure path degrades
// rather than blocks. The important one is in control_state_get_frame(): on a failed take it
// emits a ZEROED NEUTRAL FRAME rather than nothing, because the flight controller needs a steady
// 50 Hz or its own link watchdog fires. A safe frame beats no frame.
//
// MIRROR ANY CHANGE HERE IN tools/mock_server.py - it reimplements this file's math so the
// dashboard can be developed without hardware, and divergence shows up as phantom bugs.

#include "control_state.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "CONTROL_STATE";

// ---------------------------------------------------------------------------
// Axis limits. These bound what the dashboard is allowed to ask for; the flight controller
// clamps again on its side, so these are the comfortable range rather than the safety limit.
// ---------------------------------------------------------------------------
#define MAX_ROLL_PITCH_DEG 15.0f    // gentle: this is an indoor drone flying near objects
#define MAX_YAW_RATE_DPS 90.0f

// Ramp and decay rates, in units per second.
// Decay is deliberately ~2x the ramp rate: releasing a button should settle the drone faster
// than pressing it made it move.
#define ROLL_PITCH_RAMP_DPS 30.0f
#define ROLL_PITCH_DECAY_DPS 60.0f
#define YAW_RAMP_DPS2 180.0f
#define YAW_DECAY_DPS2 360.0f

// Throttle trim: persistent, no decay. 10%/second is slow enough to be controllable with a
// button and fast enough to get off the ground without a long press.
#define THROTTLE_TRIM_RATE_PER_S 0.1f
#define THROTTLE_MAX 0.85f          // headroom left for the attitude loops to mix in

// ---------------------------------------------------------------------------
// Browser watchdogs.
//
// INPUT_TIMEOUT: the dashboard posts at 20 Hz (every 50 ms). If nothing arrives for 500 ms
// the tab has been backgrounded, the phone has locked, or the Wi-Fi has dropped. Release
// every directional input so the drone stops manoeuvring and levels out. The throttle trim
// is kept, so it holds altitude rather than dropping out of the sky.
//
// DISARM_TIMEOUT: a longer backstop. If the browser has been gone for 3 seconds it is not
// coming back in time to matter, and leaving the drone hovering armed and unattended is not
// acceptable. This drops the arm request, which the flight controller acts on immediately.
//
// NOTE: the second timeout is an addition beyond the 500 ms release that was specified.
// Without it, a crashed browser tab leaves the drone hovering indefinitely, because the
// telemetry module keeps sending perfectly valid 50 Hz frames and the flight controller's own
// link watchdog never fires. Raise, lower or remove DISARM_TIMEOUT to taste.
// ---------------------------------------------------------------------------
#define INPUT_TIMEOUT_US 500000    // 500 ms
#define DISARM_TIMEOUT_US 3000000  // 3 s

typedef struct {
    control_buttons_t buttons;

    float roll_deg;
    float pitch_deg;
    float yaw_rate_dps;
    float throttle_trim;

    bool arm_request;
    bool kill_request;
    bool hold_request;
    uint8_t flight_mode;
    bool capture_pending;

    int64_t last_input_us;
    bool inputs_released;      // latch, so the release is only logged once
} control_state_t;

static control_state_t state;
static SemaphoreHandle_t state_mutex;

static telemetry_status_payload_t last_status;
static bool has_status;
static int64_t last_status_us;
static SemaphoreHandle_t status_mutex;

static float clampf(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// Ramps `value` towards `target` at `rate` units per second, without overshooting.
static float ramp_towards(float value, float target, float rate, float dt) {
    const float step = rate * dt;
    if (value < target) {
        value += step;
        if (value > target) value = target;
    } else if (value > target) {
        value -= step;
        if (value < target) value = target;
    }
    return value;
}

// Applies ramp-on-hold / decay-on-release to one bidirectional axis.
static float update_axis(float current, bool positive_held, bool negative_held,
                         float limit, float ramp_rate, float decay_rate, float dt) {
    // Both buttons held (or neither) means no commanded direction: decay to neutral.
    if (positive_held == negative_held) {
        return ramp_towards(current, 0.0f, decay_rate, dt);
    }

    const float target = positive_held ? limit : -limit;
    return ramp_towards(current, target, ramp_rate, dt);
}

esp_err_t control_state_init(void) {
    state_mutex = xSemaphoreCreateMutex();
    status_mutex = xSemaphoreCreateMutex();

    if (state_mutex == NULL || status_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create control state mutexes");
        return ESP_FAIL;
    }

    memset(&state, 0, sizeof(state));
    memset(&last_status, 0, sizeof(last_status));
    has_status = false;
    last_status_us = 0;

    state.flight_mode = TELEMETRY_MODE_ANGLE;
    state.inputs_released = true;

    ESP_LOGI(TAG, "Control state ready (max %.0f deg, %.0f deg/s yaw, throttle trim %.0f%%/s)",
             (double)MAX_ROLL_PITCH_DEG, (double)MAX_YAW_RATE_DPS,
             (double)(THROTTLE_TRIM_RATE_PER_S * 100.0f));

    return ESP_OK;
}

void control_state_set_buttons(const control_buttons_t *buttons) {
    if (buttons == NULL || state_mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;
    }

    state.buttons = *buttons;
    state.last_input_us = esp_timer_get_time();

    if (state.inputs_released) {
        state.inputs_released = false;
        ESP_LOGI(TAG, "Browser input resumed");
    }

    xSemaphoreGive(state_mutex);
}

void control_state_set_arm(bool armed) {
    if (state_mutex == NULL || xSemaphoreTake(state_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;
    }

    if (state.arm_request != armed) {
        ESP_LOGI(TAG, "Arm request: %s", armed ? "ARM" : "DISARM");
    }
    state.arm_request = armed;

    // Arming from a raised throttle trim would be rejected by the flight controller's arming
    // gate anyway, so zero the trim here and save the pilot the confusion.
    if (armed) {
        state.throttle_trim = 0.0f;
    }

    state.last_input_us = esp_timer_get_time();
    xSemaphoreGive(state_mutex);
}

void control_state_set_kill(bool kill) {
    if (state_mutex == NULL || xSemaphoreTake(state_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;
    }

    if (kill && !state.kill_request) {
        ESP_LOGW(TAG, "KILL requested from dashboard");
    }
    state.kill_request = kill;

    if (kill) {
        // Drop everything at once. The flight controller latches the kill itself; this just
        // makes sure we are not still sending an arm request alongside it.
        state.arm_request = false;
        state.throttle_trim = 0.0f;
        state.roll_deg = 0.0f;
        state.pitch_deg = 0.0f;
        state.yaw_rate_dps = 0.0f;
    }

    state.last_input_us = esp_timer_get_time();
    xSemaphoreGive(state_mutex);
}

void control_state_set_hold(bool hold) {
    if (state_mutex == NULL || xSemaphoreTake(state_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;
    }

    state.hold_request = hold;
    state.last_input_us = esp_timer_get_time();
    xSemaphoreGive(state_mutex);
}

void control_state_set_mode(uint8_t mode) {
    if (state_mutex == NULL || xSemaphoreTake(state_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;
    }

    state.flight_mode = mode;
    state.last_input_us = esp_timer_get_time();
    xSemaphoreGive(state_mutex);
}

void control_state_flag_capture(void) {
    if (state_mutex == NULL || xSemaphoreTake(state_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;
    }

    state.capture_pending = true;
    xSemaphoreGive(state_mutex);
}

// The 50 Hz integration step. Called from the UART TX task, NOT from an HTTP handler - that is
// what guarantees the ramp rates below are per-second rates and not per-POST rates.
//
// Order: watchdogs first (they can synthesise a full button release), then the three ramped axes,
// then the un-ramped throttle trim.
void control_state_update(float dt) {
    if (state_mutex == NULL || dt <= 0.0f) {
        return;
    }

    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;
    }

    const int64_t now_us = esp_timer_get_time();
    const int64_t input_age_us = now_us - state.last_input_us;

    // --- Browser watchdog ---------------------------------------------------
    if (state.last_input_us == 0 || input_age_us > INPUT_TIMEOUT_US) {
        if (!state.inputs_released) {
            ESP_LOGW(TAG, "Browser stopped posting - releasing all directional inputs");
            state.inputs_released = true;
        }
        // Treat every button as released. The axes then decay to neutral through the normal
        // path below, rather than snapping to zero, so the drone levels out smoothly.
        memset(&state.buttons, 0, sizeof(state.buttons));

        // Longer backstop: give up on the browser entirely and disarm.
        if (state.last_input_us != 0 && input_age_us > DISARM_TIMEOUT_US && state.arm_request) {
            ESP_LOGW(TAG, "Browser gone for >%d ms - dropping arm request",
                     (int)(DISARM_TIMEOUT_US / 1000));
            state.arm_request = false;
            state.throttle_trim = 0.0f;
        }
    }

    // --- Roll and pitch: ramp on hold, decay on release ----------------------
    state.roll_deg = update_axis(state.roll_deg,
                                 state.buttons.roll_right, state.buttons.roll_left,
                                 MAX_ROLL_PITCH_DEG, ROLL_PITCH_RAMP_DPS, ROLL_PITCH_DECAY_DPS, dt);

    state.pitch_deg = update_axis(state.pitch_deg,
                                  state.buttons.pitch_forward, state.buttons.pitch_back,
                                  MAX_ROLL_PITCH_DEG, ROLL_PITCH_RAMP_DPS, ROLL_PITCH_DECAY_DPS, dt);

    // --- Yaw: same treatment, but the axis is a rate rather than an angle ----
    state.yaw_rate_dps = update_axis(state.yaw_rate_dps,
                                     state.buttons.yaw_right, state.buttons.yaw_left,
                                     MAX_YAW_RATE_DPS, YAW_RAMP_DPS2, YAW_DECAY_DPS2, dt);

    // --- Throttle: persistent trim, NO decay --------------------------------
    // Note the deliberate absence of a decay branch here. When neither button is held the
    // trim simply stays where it is, which is what makes button-driven altitude control
    // possible at all.
    if (state.buttons.throttle_up && !state.buttons.throttle_down) {
        state.throttle_trim += THROTTLE_TRIM_RATE_PER_S * dt;
    } else if (state.buttons.throttle_down && !state.buttons.throttle_up) {
        state.throttle_trim -= THROTTLE_TRIM_RATE_PER_S * dt;
    }
    state.throttle_trim = clampf(state.throttle_trim, 0.0f, THROTTLE_MAX);

    // Disarmed means the trim has no business being anywhere but zero - otherwise re-arming
    // would be blocked by the flight controller's throttle-down gate and the pilot would have
    // to work out why.
    if (!state.arm_request) {
        state.throttle_trim = 0.0f;
    }

    xSemaphoreGive(state_mutex);
}

void control_state_get_frame(telemetry_control_payload_t *out) {
    if (out == NULL || state_mutex == NULL) {
        return;
    }

    memset(out, 0, sizeof(*out));

    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        // Could not read the state. Emit a safe neutral frame rather than nothing at all -
        // the flight controller needs a steady 50 Hz or its link watchdog fires.
        return;
    }

    out->roll_setpoint = state.roll_deg;
    out->pitch_setpoint = state.pitch_deg;
    out->yaw_setpoint = state.yaw_rate_dps;
    out->throttle = state.throttle_trim;
    out->armed = state.arm_request ? 1 : 0;
    out->flight_mode = state.flight_mode;

    out->flags = 0;
    if (state.kill_request) out->flags |= TELEMETRY_CTRL_FLAG_KILL;
    if (state.hold_request) out->flags |= TELEMETRY_CTRL_FLAG_HOLD;
    if (state.capture_pending) {
        out->flags |= TELEMETRY_CTRL_FLAG_CAPTURE;
        state.capture_pending = false;   // one-shot: cleared as soon as it goes out
    }

    xSemaphoreGive(state_mutex);
}

float control_state_get_throttle_trim(void) {
    if (state_mutex == NULL || xSemaphoreTake(state_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return 0.0f;
    }

    const float trim = state.throttle_trim;

    xSemaphoreGive(state_mutex);
    return trim;
}

void control_state_store_status(const telemetry_status_payload_t *status) {
    if (status == NULL || status_mutex == NULL) {
        return;
    }

    if (xSemaphoreTake(status_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;
    }

    last_status = *status;
    has_status = true;
    last_status_us = esp_timer_get_time();

    xSemaphoreGive(status_mutex);
}

bool control_state_get_status(telemetry_status_payload_t *out) {
    if (out == NULL || status_mutex == NULL) {
        return false;
    }

    if (xSemaphoreTake(status_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return false;
    }

    *out = last_status;
    const bool result = has_status;

    xSemaphoreGive(status_mutex);
    return result;
}

int32_t control_state_status_age_ms(void) {
    if (status_mutex == NULL || xSemaphoreTake(status_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return -1;
    }

    int32_t age_ms = -1;
    if (has_status) {
        age_ms = (int32_t)((esp_timer_get_time() - last_status_us) / 1000);
    }

    xSemaphoreGive(status_mutex);
    return age_ms;
}

bool control_state_browser_connected(void) {
    if (state_mutex == NULL || xSemaphoreTake(state_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return false;
    }

    const bool connected = (state.last_input_us != 0) &&
                           ((esp_timer_get_time() - state.last_input_us) < INPUT_TIMEOUT_US);

    xSemaphoreGive(state_mutex);
    return connected;
}
