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
//   ramp_towards() is the primitive. Three different policies are layered on top of it, one per
//   kind of control:
//
//   ROLL / PITCH  - analogue joystick (2026-08-29). stick_to_angle() applies an expo curve to the
//                   normalised stick position, scales it to degrees and slew-limits the result.
//                   No ramp, no decay, no press edge: the stick position IS the command.
//   YAW           - buttons through update_axis(): ramp while held, decay on release.
//   THROTTLE      - buttons, persistent TRIM with NO decay branch at all, which is what makes
//                   button-driven altitude possible.
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
// THERE IS A SECOND COPY OF THIS MATH - KEEP IT IN STEP.
//   ../../tools/mock_server.py   (that is telemetry_module/tools/mock_server.py)
// It reimplements apply_expo/stick_to_angle, update_axis, the throttle trim and both watchdogs so
// the dashboard can be developed without flashing a board. Change the integration here, or the
// shape of a /api/* response, and you must change it there too.
//
// A 2026-08-29 note here claimed that file had been deleted. It had not - it only ever lived under
// telemetry_module/tools/, and the repo-root tools/ that the note was looking at holds
// ground_station/ alone. Acting on that note is how the two drifted apart: the mock kept the old
// press-kick + ramp/decay roll/pitch long after this file moved to the joystick, so the dashboard
// behaved differently against the mock than against the drone. Do not delete this rule again.

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
#define MAX_ROLL_PITCH_DEG 10.0f    // gentle: this is an indoor drone flying near objects
#define MAX_YAW_RATE_DPS 90.0f

// ---------------------------------------------------------------------------
// ROLL / PITCH ARE NOW AN ANALOGUE JOYSTICK  (2026-08-29)
//
// WHY THE BUTTONS WERE REPLACED
//   Buttons turn a proportional quantity into a TIMING problem. To command 6 deg the pilot had to
//   hold for some particular number of milliseconds, by feel, while watching a drifting drone.
//   Two rounds of retuning the kick/ramp/decay constants widened the window but never removed the
//   underlying issue: the pilot was aiming with a stopwatch.
//
//   With a stick, thumb POSITION is the angle. 30% out is 30% of the way along the expo curve and
//   stays there for as long as you hold it. Release and it centres. There is nothing to time.
//
// WHAT THIS REPLACED
//   apply_press_kick() and the roll/pitch half of the update_axis() ramp/decay integration are
//   gone. update_axis() itself remains - yaw still uses it.
//
// WHY YAW AND THROTTLE KEEP THEIR BUTTONS
//   Same reason they never got the press-kick. Yaw commands a RATE and throttle a persistent trim;
//   both are things the pilot sets and leaves, not things aimed continuously. A second stick would
//   also need a second thumb, and the pilot has one hand on the phone.
//
// EXPO: out = (1 - E) * x + E * x^3, applied to normalised displacement before scaling to degrees.
//   The whole point is fine resolution near centre, which is where drift correction happens:
//     30% stick -> 1.4 deg      50% stick -> 2.8 deg      100% stick -> 10 deg
//   A linear stick would give 3.0 / 5.0 / 10.0 - twice as coarse exactly where it matters most.
//   Raise E for finer centre and a more aggressive edge; 0 makes the stick linear.
//
// SLEW LIMIT: bounds how fast the commanded angle may move, in deg/s. A thumb cannot move faster
//   than this, so it is invisible in normal use. It exists for the two abnormal cases: a corrupt
//   or malicious jx/jy stepping the setpoint across full scale in one frame, and the browser
//   watchdog zeroing the stick - which then eases the drone back to level instead of snapping it.
//   Full scale in ~83 ms.
// ---------------------------------------------------------------------------
#define ROLL_PITCH_EXPO 0.60f
#define ROLL_PITCH_SLEW_DPS 120.0f

// Ramp and decay rates for the YAW axis, in units per second.
//
// Roll and pitch no longer have ramp/decay constants at all - the joystick above replaced that
// whole mechanism. The history is worth keeping though, because it is why the joystick exists:
// two rounds of tuning these numbers for roll/pitch (30->45->10 deg/s ramp, a 5 deg then 4 deg
// press-kick, a 15 deg then 10 deg ceiling) each improved the feel and each left the same
// complaint, because hold-DURATION was never a quantity the pilot could aim with.
//
// Yaw keeps them because yaw is genuinely a "point it and leave it" control: decay is 2x ramp, so
// releasing settles the drone faster than pressing moved it.
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

    // Latest joystick displacement, [-1, +1] each, already clamped to the unit disc by
    // control_state_set_stick(). Zeroed by the browser watchdog exactly like the buttons are.
    float stick_x;
    float stick_y;

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

// Expo curve for the joystick: gentle near centre, full authority at the edge.
// See the block comment at ROLL_PITCH_EXPO for why, and for the numbers it produces.
static float apply_expo(float x, float expo) {
    return ((1.0f - expo) * x) + (expo * x * x * x);
}

// Turns one normalised stick axis into a commanded angle, slew-limited from its previous value.
// The slew limit is what makes a watchdog-zeroed stick ease back to level rather than snap.
static float stick_to_angle(float current_deg, float stick, float dt) {
    const float target_deg = apply_expo(clampf(stick, -1.0f, 1.0f), ROLL_PITCH_EXPO)
                             * MAX_ROLL_PITCH_DEG;
    return ramp_towards(current_deg, target_deg, ROLL_PITCH_SLEW_DPS, dt);
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

void control_state_set_stick(float x, float y) {
    if (state_mutex == NULL) {
        return;
    }

    // Reject NaN/Inf before they can reach the setpoint. isfinite() is the whole guard: a NaN
    // would propagate through the expo and the slew limit untouched (every comparison against it
    // is false, so ramp_towards() would leave it alone) and end up in the control frame.
    if (!isfinite(x)) x = 0.0f;
    if (!isfinite(y)) y = 0.0f;

    x = clampf(x, -1.0f, 1.0f);
    y = clampf(y, -1.0f, 1.0f);

    // Clamp to the unit DISC, not the unit square. A full diagonal push is sqrt(2) long, which
    // would otherwise command 1.41x the tilt of a straight push on each axis - the drone would
    // bank harder diagonally than it ever does forwards. The dashboard clamps too; this is the
    // authoritative one, because the firmware cannot trust what a client sends.
    const float magnitude = sqrtf((x * x) + (y * y));
    if (magnitude > 1.0f) {
        x /= magnitude;
        y /= magnitude;
    }

    if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;
    }

    state.stick_x = x;
    state.stick_y = y;

    // Deliberately does NOT touch last_input_us or inputs_released. control_state_set_buttons()
    // owns the browser watchdog and is called from the same handler on the same POST; having one
    // owner means there is exactly one place where "the browser is alive" is decided.

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

    // --- Does the flight controller currently say it is DISARMED? -----------
    //
    // Read BEFORE taking state_mutex, deliberately. These accessors take status_mutex, and
    // calling them while holding state_mutex would be the only place in this file that nests the
    // two - one lock-ordering mistake later and that is a deadlock on the 50 Hz task. Copying the
    // answer out first costs one extra mutex round trip per tick and removes the hazard entirely.
    //
    // "Fresh" matters as much as "disarmed". A STALE status must NOT count as disarmed: the
    // FC->module direction can drop while module->FC still works, and treating silence as
    // "disarmed" would zero the throttle trim of a drone that is still flying. Only an explicit,
    // recent "I am not armed" is trusted.
    telemetry_status_payload_t fc_status;
    const bool have_fc_status = control_state_get_status(&fc_status);
    const int32_t fc_status_age_ms = control_state_status_age_ms();
    const bool fc_status_fresh = have_fc_status && (fc_status_age_ms >= 0) &&
                                 (fc_status_age_ms < 500);
    const bool fc_reports_disarmed = fc_status_fresh && (fc_status.armed == 0);

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
        // Treat every button as released AND the stick as centred. The axes then return to
        // neutral through the normal path below - the yaw decay and the roll/pitch slew limit -
        // rather than snapping to zero, so the drone levels out smoothly.
        memset(&state.buttons, 0, sizeof(state.buttons));
        state.stick_x = 0.0f;
        state.stick_y = 0.0f;

        // Longer backstop: give up on the browser entirely and disarm.
        if (state.last_input_us != 0 && input_age_us > DISARM_TIMEOUT_US && state.arm_request) {
            ESP_LOGW(TAG, "Browser gone for >%d ms - dropping arm request",
                     (int)(DISARM_TIMEOUT_US / 1000));
            state.arm_request = false;
            state.throttle_trim = 0.0f;
        }
    }

    // --- Roll and pitch: straight from the joystick -------------------------
    // No ramp, no decay, no kick, no edge detection. The stick position IS the command; all this
    // does is shape it (expo), scale it to degrees, and bound how fast it may move. Centring the
    // stick centres the setpoint, which is why there is no "release" concept here at all.
    state.roll_deg  = stick_to_angle(state.roll_deg,  state.stick_x, dt);
    state.pitch_deg = stick_to_angle(state.pitch_deg, state.stick_y, dt);

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

    // --- Trim must be zero whenever the drone is not actually flying --------
    //
    // BUGFIX 2026-08-29. This used to test only `!state.arm_request`, which left a lockout the
    // pilot could walk into and never get out of:
    //
    //   1. Something disarms the drone (link blip, watchdog, kill, a refused arm).
    //   2. Pilot presses ARM. arm_request goes true, so the trim STOPS being zeroed - even
    //      though the flight controller has not armed and may be refusing to.
    //   3. Nothing happens, so the pilot presses ALT+ to make it move. The trim climbs.
    //   4. The FC's arming gate requires throttle < ARM_THROTTLE_THRESHOLD (0.02). The trim is
    //      now above it, so arming is refused - permanently, and more ALT+ makes it worse.
    //
    // The pilot sees the trim counter going up, "Armed" stuck at no, and dead motors, with no
    // indication that the two are related. The only escape was ALT- back to zero, which is not
    // something anyone would guess.
    //
    // Testing the FLIGHT CONTROLLER's own armed flag instead of the pilot's request closes it:
    // trim cannot climb until the drone is genuinely armed, so the throttle-down gate can always
    // be satisfied. Note fc_reports_disarmed is false when the status is stale, so a status
    // dropout mid-flight does NOT cut the throttle of a drone that is still flying.
    if (!state.arm_request || fc_reports_disarmed) {
        state.throttle_trim = 0.0f;
    }

    // (The prev_buttons edge reference that used to live here went with the press-kick. Nothing
    // in this file is edge-triggered any more: yaw and throttle care only about what is held
    // right now, and roll/pitch read an absolute stick position.)

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
