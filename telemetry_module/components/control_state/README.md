# `control_state` — buttons into setpoints

The pilot's dashboard has **no joysticks**. It has buttons you press and hold. This component is
what turns "the roll-right button is currently down" into "roll setpoint = 11.3°" — and it is the
only place on either board where that synthesis happens.

It also holds the last status frame received from the flight controller, so the HTTP layer has
something to serve to the dashboard without touching the UART.

## Ramp on hold, decay on release

While a button is held, its axis **ramps** toward the axis limit. When released, it **decays**
back to neutral:

```
  held    →  ramp_towards(±limit,  ramp_rate,  dt)
  released→  ramp_towards(   0,   decay_rate, dt)
```

**Decay is deliberately about 2× the ramp rate.** Letting go should always settle the drone faster
than pressing made it move — that asymmetry is what makes a button interface flyable at all.

`update_axis()` treats "both buttons held" identically to "neither held": no commanded direction,
so decay to neutral. That avoids having to define what left+right simultaneously means.

| Axis | Limit | Ramp | Decay |
|---|---|---|---|
| Roll / pitch | ±15° | 30 °/s | 60 °/s |
| Yaw **rate** | ±90 °/s | 180 °/s² | 360 °/s² |

15° is deliberately gentle — this is an indoor drone flying near objects. These bound what the
dashboard may *ask* for; the flight controller clamps again on its side (±25°), so these are the
comfortable range rather than the safety limit.

Yaw is a **rate**, not an angle, because there is no magnetometer on the airframe — see
[`attitude_estimator`](../../../flight_controller/components/attitude_estimator/README.md).

## Throttle is the exception — a trim, not a stick

```c
if (throttle_up)        trim += RATE * dt;
else if (throttle_down) trim -= RATE * dt;
// note the deliberate absence of an `else` decay branch
```

Holding "up" raises the trim; releasing leaves it **exactly where it was**. A decaying throttle
would mean the drone sinks the moment you stop pressing, which is unflyable with buttons.

`THROTTLE_TRIM_RATE_PER_S` is `0.1f` — 10 % per second, clamped to `THROTTLE_MAX` (0.85, leaving
headroom for the attitude loops to mix in).

> This value was lowered from 0.25 in a later commit ("changed throttle increment"). If you change
> it again, the ramp feel changes for the pilot and `tools/mock_server.py` needs the same edit.

The trim is forcibly zeroed in three places, all for the same reason: the flight controller's
arming gate refuses to arm on a raised throttle, and a pilot who hits that would have no idea why.

1. `control_state_set_arm(true)` — zero it on the way in.
2. `control_state_update()` — whenever `!arm_request`, hold it at zero.
3. `control_state_set_kill(true)` — along with everything else.

## The two browser watchdogs

The dashboard posts button state at 20 Hz (every 50 ms). Two separate timeouts guard against it
going away.

### `INPUT_TIMEOUT_US` — 500 ms → release all directional inputs

The tab was backgrounded, the phone locked, or the Wi-Fi dropped. Every button is treated as
released and the axes **decay to neutral through the normal path** rather than snapping to zero —
so the drone levels out smoothly instead of jerking.

The throttle trim is deliberately **kept**, so the drone holds altitude rather than dropping out
of the sky.

### `DISARM_TIMEOUT_US` — 3 s → drop the arm request

The longer backstop, and the one that is easy to overlook the need for. Without it, a crashed
browser tab leaves the drone hovering **indefinitely** — because the telemetry module keeps
sending perfectly valid 50 Hz control frames, so the flight controller's own 300 ms link watchdog
never fires. The link is fine; it is the *pilot* who is gone.

At 3 s the arm request is dropped and the trim zeroed. The flight controller acts on that
immediately.

> This second timeout is an addition beyond the 500 ms release that was originally specified.
> Raise, lower or remove it to taste — but understand what it is protecting against first.

`state.inputs_released` is a latch so the release is logged once rather than at 50 Hz.

## Why `control_state_update()` runs from the UART task

It is called at a **fixed 50 Hz from the UART TX task**, not from the HTTP handler.

Setpoints must advance at a known rate regardless of how irregularly the browser posts. If the
integration were driven by the POST handler, a browser that stuttered from 20 Hz down to 5 Hz
would quietly change every ramp rate by 4×. The HTTP handler only *records* button state and
resets the watchdog; the integration is on its own clock.

## Public API

### Input side — called from HTTP handlers

| Function | Notes |
|---|---|
| `control_state_set_buttons(&b)` | Records held buttons, resets the watchdog. |
| `control_state_set_arm(bool)` | Also zeroes the throttle trim on arm. |
| `control_state_set_kill(bool)` | Also drops arm, trim, and all three axes at once. |
| `control_state_set_hold(bool)` | Altitude/position hold toggle. |
| `control_state_set_mode(uint8_t)` | `telemetry_flight_mode_t`. |
| `control_state_flag_capture()` | One-shot; cleared as soon as it goes out in a frame. |

All of these also refresh `last_input_us` — any dashboard interaction counts as the browser being
alive, not just directional POSTs.

### Output side — called from the UART TX task

| Function | Notes |
|---|---|
| `control_state_update(dt)` | Integration + watchdogs. Fixed 50 Hz. |
| `control_state_get_frame(&out)` | Fills a `telemetry_control_payload_t`. |

### Status side

| Function | Notes |
|---|---|
| `control_state_store_status(&s)` | Called from the UART RX task. |
| `control_state_get_status(&out)` | Returns false if the drone has never replied. |
| `control_state_status_age_ms()` | −1 if no status has ever arrived. |
| `control_state_browser_connected()` | Within `INPUT_TIMEOUT_US`. |
| `control_state_get_throttle_trim()` | So the dashboard can show what the pilot has actually asked for, not just what they last pressed. |

## Timing and concurrency

**Two independent mutexes**, and the split is deliberate: `state_mutex` for the control state,
`status_mutex` for the last received status frame. The HTTP handler reading status for
`/api/status` must not contend with the UART TX task integrating setpoints — those are different
tasks on different clocks touching disjoint data.

Every take uses a **5 ms timeout**, never `portMAX_DELAY`. On failure each function degrades
rather than blocking:

- Setters: silently drop the update. The browser will post again in 50 ms.
- `control_state_update()`: skip this integration step.
- `control_state_get_frame()`: **emit a zeroed, safe, neutral frame** rather than nothing at all —
  the flight controller needs a steady 50 Hz or its own link watchdog fires. Sending a neutral
  frame is strictly better than sending none.

`capture_pending` is a one-shot cleared inside `get_frame()`, so exactly one outgoing frame
carries the capture bit no matter how the HTTP request and the 50 Hz cadence line up.

## ⚠ Keep `tools/mock_server.py` in sync

`telemetry_module/tools/mock_server.py` mirrors this file's setpoint integration, the arming gate
and both watchdogs **byte-for-byte**, so the dashboard can be developed with no hardware attached.

**If you change the ramp/decay math, a limit, or a timeout here, change it there too** — otherwise
dashboard behaviour silently diverges between the mock and the real board, and you will debug the
difference rather than the bug.

## Dependencies

**Public** (`REQUIRES`): `telemetry_uart` — `telemetry_control_payload_t` and
`telemetry_status_payload_t` are in this component's API.
**Private** (`PRIV_REQUIRES`): `esp_timer`.

Consumed by `uart_link` (drives the 50 Hz update and reads frames) and `http_server` (sets
buttons, reads status).

## Files

| File | Contents |
|---|---|
| `include/control_state.h` | `control_buttons_t`, thirteen public functions, the ramp/decay contract. |
| `control_state.c` | Limits and rates, both watchdogs, `update_axis()`, the two mutexes, frame assembly. |
| `CMakeLists.txt` | Component registration. |
