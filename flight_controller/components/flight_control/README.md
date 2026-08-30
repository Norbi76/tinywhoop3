# `flight_control` — the cascade, the mixer, arming, and the link watchdog

The orchestrator. Everything else on the flight controller either feeds this component or is
driven by it. It owns the eight PID instances, decides when motors are allowed to run, converts
three axis commands into four motor thrusts, and disarms the aircraft when the pilot's link goes
away.

`flight_control_update()` is called at exactly 1 kHz from `fc_task` and is the only thing that
writes the motors in normal operation.

## The cascade

Three nested loops, all clocked off the same 1 kHz tick using **divider counters** rather than
separate tasks — so they can never drift in phase relative to one another:

```
  OUTER    50 Hz   every 20 ticks   altitude  -> throttle
                                    velocity  -> angle setpoints
     ↓
  MID     250 Hz   every  4 ticks   angle     -> rate setpoints
     ↓
  INNER  1000 Hz   every    tick    rate      -> mixer -> motors
```

Why this shape:

- **Inner** needs to be fast — it is what actually stabilises the airframe, and it only needs the
  gyro, which is available every tick anyway.
- **Mid** at 250 Hz because angles change far more slowly than rates; running it at 1 kHz would
  cost cycles and gain nothing.
- **Outer** at 50 Hz because it is bounded by the sensors behind it (ToF at ~33 Hz, flow at
  100 Hz), and because altitude and position are slow quantities.

**Only the outer loop depends on `nav_estimator`.** It disengages itself the moment
`altitude_valid` or `velocity_valid` goes false. The mid and inner loops need only the IMU. That
is what makes the drone flyable in angle mode with no ToF and no optical flow fitted at all, and
what makes outer-loop hardware failures degrade gracefully instead of faulting.

### The eight PID instances

| Loop | Rate | Input → Output | Output limit | I-limit | D cutoff |
|---|---:|---|---|---|---|
| `RATE_ROLL` / `RATE_PITCH` / `RATE_YAW` | 1000 Hz | °/s → mix contribution | ±0.5 | 0.2 | 60 Hz |
| `ANGLE_ROLL` / `ANGLE_PITCH` | 250 Hz | ° → °/s | ±250 °/s | 50 | 30 Hz |
| `VEL_X` / `VEL_Y` | 50 Hz | m/s → ° | ±25° | 10 | 10 Hz |
| `ALTITUDE` | 50 Hz | m → throttle adjust | ±0.25 | 0.25 | 10 Hz |

The rate loops' ±0.5 output range is deliberately narrow: a single axis can never claim more than
half the motor range, which leaves room for the other two axes and the throttle. The D cutoffs
step down as you go outward — the rate loop sees raw gyro and needs the most filtering.

The array `pid_loops[]` is indexed by **`telemetry_loop_id_t`** (the web dashboard's ordering).
The ground station uses **`pid_loop_id_t`** (the opposite ordering). The `pid_loop_to_telemetry[]`
table near the top of `flight_control.c` is the single point where those two meet — nothing else
in the file needs to know both orderings exist.

### `flight_control_run_pid()`

Every PID call in the cascade goes through this one wrapper, which:

1. adds `pid_registry_inject_offset()` to the setpoint (the ground station's test signal), then
2. calls `pid_update()`, then
3. calls `pid_registry_publish()` to sample the controller's internals.

Both registry calls are non-blocking by construction — no lock, no UART, no logging — which is
what makes this safe in the 1 kHz path as well as the slower ones. `now_s` is taken **once** per
`flight_control_update()` so all three loops evaluate the same point on the injection waveform
within one tick.

## The mixer

### ⚠ Every sign in `flight_control_mix()` needs bench verification

This is the single most likely place for the drone to flip on its first flight.

```
        FRONT
   M0 ........ M1        M0 front-left  (CW)    M1 front-right (CCW)
   M2 ........ M3        M2 rear-left   (CCW)   M3 rear-right  (CW)
        REAR

  mix[M0] =  roll - pitch - yaw
  mix[M1] = -roll - pitch + yaw
  mix[M2] =  roll + pitch + yaw
  mix[M3] = -roll + pitch - yaw
```

The reasoning behind each column:

- **Roll** (+ = roll right, right side down): lift the **left** side → M0, M2 up / M1, M3 down.
- **Pitch** (+ = nose up): lift the **rear** → M2, M3 up / M0, M1 down.
- **Yaw** (+ = nose right): a motor spinning CW drags the frame CCW, so to rotate the frame CW we
  speed up the **CCW** motors → M1, M2 up / M0, M3 down.

**Bench check — props OFF, armed, a little throttle:**

| Command | Motors that must speed up |
|---|---|
| Roll right | M0 and M2 |
| Pitch up | M2 and M3 |
| Yaw right | M1 and M2 |

If an axis is backwards, flip the sign of that whole column. If the prop directions on your build
differ from the CW/CCW above, the **yaw column** is the one that changes.

### The three-step throttle placement

The mixer does not simply add throttle to the mix and clamp. That naïve version has a specific
failure mode this code is arranged to avoid: **a brushed motor commanded to zero has stopped, and
a stopped motor produces no control torque at all until it spins back up.** Clipping the low
motors to zero therefore destroys the attitude command precisely when the drone most needs it.

**Step 1 — fit the attitude command into the available band.** The usable band is
`[MOTOR_IDLE_THRUST, 1.0]`. If the spread the attitude loops asked for is wider, scale the **mix
only** down until it fits. Scaling the *differences* preserves the commanded attitude; scaling the
motor outputs (throttle included) would not, because it shrinks the differences by a different
proportion than it shrinks the average.

**Step 2 — place the throttle so nothing falls off either end.** Rather than clipping, shift the
whole group up so the lowest motor lands on the idle floor. That upward shift is capped at
`MAX_MIXER_THROTTLE_BOOST` (15 %) — without a cap this becomes full "airmode", and on a
button-driven throttle that means holding ALT− stops producing a descent. The cap is a deliberate
trade: pilot authority over the descent, at the cost of some attitude authority at very low stick.

**Step 3 — if the capped boost left us short, shrink the mix to fit.** `fit` scales all three axes
equally, so the *direction* of the commanded attitude is preserved and only its magnitude shrinks.
Authority degrades smoothly instead of two motors dropping out.

The final `clampf(..., 0, 1)` should never bite after those three steps. It is a safety net
against a NaN or a future edit, not load-bearing behaviour.

## Arming state machine

```
  DISARMED ──(arm requested + all gates pass)──> ARMED
     ↑                                             │
     └──(disarm / link lost / kill cleared)────────┘
     ↑
  KILLED ──(kill released AND arm released)────────┘
```

### Kill is absolute and latching

`TELEMETRY_CTRL_FLAG_KILL` immediately cuts motors and sets `kill_latched`. The latch clears only
when the pilot has **released kill AND is no longer asking to arm** — so letting go of the kill
button cannot by itself spin the motors back up. Kill is checked before the link watchdog and
before everything else.

### The arming gate — all four must hold

1. **No kill latch.**
2. **Link OK** — a valid control frame within the last 300 ms.
3. **Attitude initialised** — `attitude->initialized`, i.e. the complementary filter has settled.
   Normal to fail for the first half-second after boot.
4. **Throttle genuinely down** — below `ARM_THROTTLE_THRESHOLD` (0.02). This is what stops the
   drone leaping off the bench the instant it is armed.

This is the **only** path into `FLIGHT_STATE_ARMED`.

### Integrators are reset on arm *and* disarm

`flight_control_reset_all_pids()` runs on both. On disarm because whatever the integrators wound
up to while the drone was being wrestled to the ground is meaningless; on arm because time may
have passed and it is the last chance to guarantee a clean start.

### The single most important line in the file

```c
if (!motors_allowed) {
    flight_control_stop_motors();
    tick_counter = 0;
    return;
}
```

Every path that does not *explicitly* authorise the motors ends up here. Structuring it as a
single early return — rather than scattering conditionals through the cascade — is what makes
"disarmed means stopped" auditable.

## Disarmed monitor mode

`FLIGHT_CONTROL_MONITOR_WHEN_DISARMED` (default 1) runs the **outer loop only** while disarmed,
after the motors have already been stopped by the invariant above and before the early return. No
mid loop, no inner loop, no mixer, no motor write — the outer loop's only outputs are
`angle_setpoint_*` and `throttle_command`, and nothing that consumes those is running.

It exists to make the **unverified velocity→angle sign checkable without flying**. A wrong sign
there is positive feedback: the drone accelerates away from the hold point instead of settling.
Before this, the only way to find out was to engage position hold in the air.

### The bench procedure

Props off, drone powered, **disarmed**, ground station connected, PID tab on `Vel X` (then
`Vel Y`). Hold the drone 20–50 cm above a textured floor — on a desk the ToF sits outside its
0.03–1.30 m window, so `altitude_valid` is false and nothing computes. Press **HOLD** on the
dashboard, move the drone by hand, and read the published `output`:

| Move the drone | Loop | `output` must be | Because |
|---|---|---|---|
| Forward | `Vel X` | **negative** | negated at the call site → positive pitch = nose up = decelerate |
| Right | `Vel Y` | **negative** | used directly → negative roll = left wing down = decelerate |

The rule is the same for both axes: **move in the positive direction, the output goes negative.**
If it goes positive, that axis' sign is wrong, and in flight it is positive feedback.

> ⚠ `Vel X` is negated *after* `flight_control_run_pid()` returns, so the `output` in the PID trace
> is the value **before** negation. `Vel Y` is not negated. Read the table, not your intuition.

The Mode tile is live here too: it reads `POS HOLD` only when the loops genuinely engaged, so this
doubles as a way to see whether hold *would* engage at all without leaving the ground.

### Why it cannot reach the motors

`flight_control_stop_motors()` is called before it, and the function returns immediately after, so
no code path between the monitor and the motor driver is reachable. Integrators wound up while
monitoring are cleared by `flight_control_reset_all_pids()` on the way into `ARMED`, which also
clears both hold latches. It does not run while the kill latch is set.

It uses its own `monitor_tick_counter` rather than `tick_counter`, because the disarm path zeroes
that one every tick — sharing it would pin the monitor at "tick 0", running the outer loop at the
full 1 kHz while still passing `OUTER_LOOP_DT` (0.020 s) as its `dt` and making every rate in the
published trace wrong by 20×.

Set the macro to 0 for a build where a disarmed aircraft computes nothing at all.

### Idle cutoff

Armed but throttle below `MOTOR_IDLE_THRESHOLD` (0.05), and not in altitude hold: motors off,
integrators cleared. This stops the controller winding up against the ground before takeoff.

### Why `tick_counter = 0` on both stop paths

The divider counters are tested **before** they are incremented, so the very first tick after
arming (`counter == 0`) runs all three loops. Without the reset, `throttle_command` would still
hold the 0 left by `flight_control_reset_all_pids()` for the first 20 ticks, and the mixer's idle
floor would spin the motors on a throttle the pilot never commanded.

## Control-link watchdog

`LINK_TIMEOUT_US` is 300 ms. The telemetry module sends control frames at a fixed 50 Hz (20 ms),
so 300 ms tolerates **15 consecutive lost frames** — long enough to ride out a Wi-Fi hiccup, short
enough to matter if the pilot walks out of range.

`last_control_input_us == 0` (nothing has *ever* arrived) is explicitly not "link ok", so a board
that boots with no telemetry module attached can never arm.

`flight_control_set_control_input()` is called from `telemetry_task` on **core 0** while
`flight_control_update()` reads on **core 1**. The copy is done inside a `portMUX` critical
section on both sides — a torn read would mean acting on half of one frame and half of another,
e.g. the new throttle with the old kill flag.

## Flight modes

| Mode | Altitude | Roll/pitch sticks |
|---|---|---|
| `ANGLE` | Pilot throttle passed straight through | Angle setpoints directly |
| `ALT_HOLD` | Held by the altitude PID; stick becomes climb/descend around centre | Angle setpoints directly |
| `POS_HOLD` | Held | **Velocity** setpoints — centred sticks ask for zero velocity, which is what makes the drone hold station |

`TELEMETRY_CTRL_FLAG_HOLD` (the dashboard's hold toggle) requests both.

On engaging altitude hold, the current height is captured as the target and the altitude PID is
reset so it does not inherit anything from a previous engagement. In hold, throttle stick offset
beyond ±0.1 from centre moves the setpoint at up to `MAX_CLIMB_RATE_MS` (0.5 m/s), clamped to
0.05–1.20 m.

> **Velocity sign, unverified:** forward velocity error is corrected by pitching, and positive
> pitch is nose-up which moves the drone *backward* — hence the negation on `PID_LOOP_VEL_X`.
> Marked `SIGN NEEDS BENCH VERIFICATION`. If the drone runs away instead of holding station, this
> is the first thing to flip.

## Constants that must be measured

| Constant | Current | Why it matters |
|---|---|---|
| `MOTOR_IDLE_THRUST` | 0.12 | The floor every motor is kept at while armed. Must be above the duty at which the motor reliably starts. **Measure:** props off, arm, walk each motor up with `motor_set_thrust()` until all four start reliably from standstill; take the worst, add margin. |
| `HOVER_THROTTLE` | 0.45 | The altitude loop's output is added on top of this. A bad value means the loop spends its whole range correcting a constant offset. **Measure:** fly in angle mode, trim until it neither climbs nor sinks, read it off the dashboard. Expect 0.35–0.55 for a 1S whoop, depending on all-up weight with the camera. |
| All 12 default gains | see table | **Starting points, not answers.** Tune inside-out: rate loops first (P until it oscillates, back off ~30 %, then D, then a little I), then angle (P only to begin with), then velocity and altitude last. All are live-tunable from the dashboard. |

Other bounds — `MAX_ANGLE_SETPOINT_DEG` (25°), `MAX_RATE_SETPOINT_DPS` (250 °/s),
`MAX_ALTITUDE_THROTTLE_ADJUST` (0.25), `MAX_MIXER_THROTTLE_BOOST` (0.15) — bound how hard each
loop may push the one below it. They are conservative by design.

## Public API

| Function | Called from | Notes |
|---|---|---|
| `flight_control_init()` | `fc_task` startup | Inits all 8 PIDs, binds them to `pid_registry`, enters DISARMED. |
| `flight_control_update(&att, &nav, dt)` | `fc_task`, 1 kHz | The whole cascade. `nav` may be NULL. |
| `flight_control_set_control_input(&ctrl)` | `telemetry_task` | Spinlock-protected, cross-core safe. |
| `flight_control_get_status(&out)` | `telemetry_task` | Fills the status frame. |
| `flight_control_is_armed()` / `_get_state()` | anywhere | |
| `flight_control_set_gains(id, kp, ki, kd)` | `telemetry_task` | Dashboard tuning. **Preserves the integrator** (unlike `pid_registry_apply_gains()`). |
| `_set_rate_gains` / `_set_angle_gains` / `_set_velocity_gains` / `_set_altitude_gains` | | Convenience pairs — those axes are almost always tuned together. |

### Two notes on `flight_control_get_status()`

- `battery_voltage` is a **fixed 3.8 V**, not a measurement. There is no ADC divider on the pack
  and none is planned; it exists to keep the dashboard field populated. It will not fall as the
  pack drains, and nothing in the control path reads it.
- `loop_hz` is the **measured** inner-loop rate, counted over a 1-second window. It is there so
  you can confirm on the bench that this really is running at 1 kHz.

## Timing and concurrency

- `flight_control_update()` runs on **core 1** in `fc_task` at the highest priority. It must never
  block: no I/O, no mutexes, no logging in the hot path. The one log inside
  `flight_control_inner_loop()` (failed motor write) is rate-limited to once per second.
- `flight_control_set_control_input()` runs on **core 0** — the only cross-core write, protected
  by `control_input_spinlock`.
- `nav_state_t` arrives already copied out of `sensor_task`'s mutex by the caller, so this
  component never touches that lock.
- Live gain setters run on `telemetry_task` and write the PID structs directly with no lock. That
  is a tolerated race: a `float` write is atomic on this target, and the worst case is one
  iteration using a mixed gain set. The ground-station path (`pid_registry_apply_gains()`) *does*
  take a mutex, because it writes six coupled fields plus the integrator.

## Unverified assumptions and TODOs

| Marker | Location | What must be checked |
|---|---|---|
| `EVERY SIGN IN THIS FUNCTION NEEDS BENCH VERIFICATION` | `flight_control_mix()` | The mixer matrix — props off, per-axis |
| `SIGN NEEDS BENCH VERIFICATION` | `flight_control_outer_loop()` | Velocity→pitch negation |
| `TODO(bench)` | `MOTOR_IDLE_THRUST` | Measure the reliable-start duty |
| `TODO(bench)` | `HOVER_THROTTLE` | Measure the hover point |
| `TODO(tune)` | 12 default gains | Tune inside-out |

Upstream, and equally load-bearing: the gyro axis mapping in
[`attitude_estimator`](../attitude_estimator/README.md) and the prop CW/CCW directions in
[`motor_driver`](../motor_driver/README.md). A mixer sign error and an axis-mapping sign error
look identical from the outside — verify the estimator first, then the mixer.

## Dependencies

**Public** (`REQUIRES`): `pid_controller`, `motor_driver`, `attitude_estimator`, `nav_estimator`,
`telemetry_uart`, `esp_timer`.
**Private** (`PRIV_REQUIRES`): `pid_registry` — nothing in `flight_control.h` mentions it, because
the tuning registry is an implementation detail of the cascade.

## Files

| File | Contents |
|---|---|
| `include/flight_control.h` | `flight_state_t`, the cascade diagram, ten public functions. |
| `flight_control.c` | Loop rates, safety constants, default gains, the loop-id bridge table, the mixer, the arming machine, the three loops, status assembly, gain setters. |
| `CMakeLists.txt` | Component registration with the public/private dependency split explained. |
