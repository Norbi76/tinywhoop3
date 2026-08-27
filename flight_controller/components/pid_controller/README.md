# `pid_controller` — the generic PID primitive

One PID implementation, instantiated **eight times** by `flight_control` (3 rate + 2 angle +
2 velocity + 1 altitude). Nothing in this component knows anything about quadcopters — it is a
plain scalar controller with no globals, no tasks and no ESP-IDF dependencies beyond `esp_err_t`
and logging.

## Why it isn't a textbook PID

A naïve `kp*e + ki*∫e + kd*de/dt` will fly badly on a small quad in four specific and
well-understood ways. All four are addressed here.

### 1. Derivative on **measurement**, not on error

```
d(error)/dt  ==  d(setpoint)/dt  -  d(measurement)/dt
                 ^^^^^^^^^^^^^^ dropped
```

Only the measurement half is used, with the sign flip that leaves: `-d(measurement)/dt`.

When the pilot slams a stick, the setpoint steps. Differentiating that step gives an enormous
spike ("setpoint kick") and a violent motor jerk that has nothing to do with what the airframe is
actually doing. Dropping the setpoint term removes it entirely, at no cost — the P term already
responds to setpoint changes.

### 2. Low-pass filtered derivative

```
RC    = 1 / (2π · d_cutoff_hz)
alpha = dt / (RC + dt)
d_filtered += alpha · (d_raw - d_filtered)
```

Differentiation amplifies high-frequency noise, and on a whoop the high-frequency noise is frame
vibration from the props. Unfiltered, that goes straight into the D term and out to the motors —
the classic cause of hot motors and a "buzzing" quad. `alpha` near 1 is almost no filtering, near
0 is very heavy.

`d_alpha` is computed once from `nominal_dt` at init. Note that it is **not** recomputed per call
even though `pid_update()` takes a real `dt` — the filter coefficient assumes the nominal rate.
That is fine while jitter is small (the loops here are phase-locked to a 1 kHz tick), but it does
mean a loop running at a significantly different rate than it was initialised for gets a different
effective cutoff than the one requested.

`d_cutoff_hz` and `nominal_dt` are retained in the struct so a gain change can recompute `d_alpha`
and so the tuning UI can read the cutoff back — `d_alpha` alone is not invertible without the
period, and there is nowhere else the period is stored.

### 3. Conditional-integration anti-windup

The integrator is the term that keeps accumulating when the output can't deliver what's asked.
Hold a drone on the bench with throttle up: the altitude loop sees a persistent error, winds up,
and the moment you let go it lurches.

The implementation computes what the output *would* be if this sample were integrated, then
freezes the integrator only when committing it would push **further into** saturation:

```c
candidate_integrator = integrator + ki·error·dt
candidate_output     = p_term + candidate_integrator + d_term

if (candidate_output > out_max && error > 0)   freeze
if (candidate_output < out_min && error < 0)   freeze
```

Crucially the integrator is *not* frozen when the error has reversed sign — the loop can always
unwind, it just can't dig deeper. On top of that there is a hard `integrator_limit` clamp.

### 4. The integrator stores the **ki-scaled** term

`integrator` accumulates `Σ ki·error·dt`, not `Σ error·dt`.

This exists for live tuning. If the raw error integral were stored, changing `ki` from the
dashboard mid-flight would instantly rescale everything accumulated so far, and the aircraft would
jolt. Storing the scaled term means a `ki` change only affects *future* contributions.
`pid_set_gains()` therefore deliberately leaves the integrator alone.

## Telemetry fields

Everything from `last_p` down is written by `pid_update()` and never read back by the control law.
It exists so `pid_registry` can publish a complete picture of one iteration without the calling
loop having to hand it the setpoint and measurement separately:

| Field | Meaning |
|---|---|
| `last_p` / `last_i` / `last_d` | The three term contributions. Lets you see which one is doing the work. |
| `last_setpoint` / `last_measurement` | As passed in — injection offset already included. |
| `last_output` | Post-clamp return value, i.e. what the next stage actually got. |
| `out_saturated` | The output clamp actually bit. |
| `integrator_clamped` | Anti-windup froze the integrator, or it hit `integrator_limit`. |

Those last two are worth the two bytes: both conditions are computed inside `pid_update()` and
would otherwise be discarded, and "why is this loop not responding" is almost always one of them.

## Public API

| Function | Notes |
|---|---|
| `pid_init(...)` | Zeroes the struct, stores gains and limits, derives `d_alpha`. Validates: non-NULL, `dt > 0`, `cutoff > 0`, `out_min < out_max`. |
| `pid_update(pid, setpoint, measurement, dt)` | One iteration. Returns the **clamped** output. Returns 0 on NULL or non-positive `dt`. |
| `pid_reset(pid)` | Clears integrator + derivative history, **keeps gains**. |
| `pid_set_gains(pid, kp, ki, kd)` | In-place gain change, **keeps the integrator**. |

### `pid_reset()` must be called on every arm and every disarm

A stale integrator is the classic cause of a quad flipping the instant it is armed. `flight_control`
does this in its arming state machine — if you add a new PID instance anywhere, wire it into that
reset path.

### The `first_update` flag

On the very first `pid_update()` after init or reset there is no previous measurement, so
`d(measurement)/dt` is meaningless. Rather than differentiating against a zero that was never a
real sample (which would produce a huge spurious D spike), the first call seeds
`prev_measurement` and emits `d_raw = 0`.

## Timing and concurrency

- Pure arithmetic on a caller-supplied struct. No allocation, no I/O, no blocking, no static
  state whatsoever — every instance's state lives in the `pid_controller_t` the caller owns.
- Runs in `fc_task` at 1 kHz (rate loops), 250 Hz (angle) and 50 Hz (outer).
- **Thread safety is the caller's problem.** Nothing here locks. `flight_control` owns all eight
  instances and only ever touches them from `fc_task`; live gain updates arriving on
  `telemetry_task` are handed across through `pid_registry`'s queue rather than by writing the
  structs directly. That indirection exists precisely because this component has no locking.
- `dt` is a parameter, so the loop rate can vary without corrupting the integral — except for
  `d_alpha`, as noted above.

## Dependencies

None. `esp_err.h` and `esp_log.h` only.

## Files

| File | Contents |
|---|---|
| `include/pid_controller.h` | `pid_controller_t` with the rationale for each non-obvious field, four public functions. |
| `pid_controller.c` | Init/validation, the control law, reset, gain update. |
| `CMakeLists.txt` | Component registration (no dependencies). |

## See also

- [`pid_registry`](../pid_registry/README.md) — cross-task gain tuning and debug streaming for
  these instances.
- [`flight_control`](../flight_control/README.md) — the eight instances, their gains, their units
  and how they cascade.
