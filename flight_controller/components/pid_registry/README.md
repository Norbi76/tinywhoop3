# `pid_registry` — live PID tuning and instrumentation

Sits between the control cascade and the telemetry link so neither has to know about the other.
`flight_control` binds its eight `pid_controller_t` instances here once at init; from then on the
registry can read and write their gains, sample their internals into a debug stream, and hand the
control loop a test-signal offset to add to a setpoint.

This is what makes the Python ground station in [`tools/ground_station`](../../../tools/ground_station/)
work: tune a loop, watch its step response, retune, without reflashing.

## The one rule this component exists to enforce

> **Everything called from `fc_task` is non-blocking.**

`pid_registry_publish()` and `pid_registry_inject_offset()` run inside the 1 kHz control task.
Neither takes a lock, touches the UART, or logs. Publishing is a **zero-timeout queue send that
drops the sample if the queue is full** — a dropped plot point costs nothing, a late control
iteration costs the drone.

The mutex protects the gain tables against writers on the **telemetry task only**. `fc_task` never
takes it.

## Which side calls what

```
  telemetry_task (50 Hz)                    fc_task (1 kHz / 250 Hz / 50 Hz)
  ──────────────────────                    ─────────────────────────────────
  pid_registry_apply_gains()   ── mutex ──> the pid_controller_t structs
  pid_registry_read_gains()    ── mutex ──>
  pid_registry_set_stream()    ── volatile bytes ──>
  pid_registry_set_inject()    ── volatile ──>
                                            pid_registry_inject_offset()  (lock-free read)
  pid_registry_pop_debug()  <── queue ──    pid_registry_publish()        (zero-timeout send)
```

### Why some shared state is lock-free

`g_stream_loop` and `g_stream_divider` are single aligned `volatile uint8_t`. A concurrent write
can only ever be observed as the old value or the new one, never a mixture, so the worst case is
one extra or one missing sample around a retune. Same reasoning for the injection parameters: a
torn read costs at most one distorted sample of an operator-triggered test signal. Neither is
worth putting a lock in the 1 kHz path for.

`pid_registry_set_stream()` writes the **divider before the loop id** so a new selection never
goes live for an instant with the previous divider still in place.

## Gain application

### `apply_gains()` zeroes the integrator; `pid_set_gains()` doesn't

That is the entire reason this function exists rather than just calling
`pid_controller`'s own setter.

The integrator holds the **ki-scaled** accumulation. Raise `ki` mid-flight and whatever was
accumulated under the old gain is still sitting in the output — as a step. On a 50 g quad with an
altitude loop, that step is the ceiling. So a full gain set from the ground station resets it.

`pid_set_gains()` keeps the integrator on purpose, for the web dashboard's small-nudge tuning. Two
callers, two different correct behaviours.

### The three limit fields treat zero as "leave this alone"

`i_limit`, `out_limit` and `d_cutoff_hz` are only applied when `> 0`. `kp`/`ki`/`kd` are **not** —
a zero gain is a meaningful thing to ask for.

This asymmetry is a safety property, not a convenience. A zero `out_limit` would clamp the loop's
output to zero for the rest of the flight; on a rate loop that is the axis gone. Nothing
legitimately asks for a zero limit, but plenty of things produce one by omission — a gain profile
carrying only kp/ki/kd, or an apply that lands before a read-back has filled the limit fields in.

A zero or negative `d_cutoff_hz` would divide by zero, so it leaves the existing filter alone
rather than producing a NaN inside the 1 kHz loop. Recomputing `d_alpha` needs the loop period,
which is why `pid_controller_t` retains `nominal_dt`.

> **Known asymmetry limitation:** the payload carries one *symmetric* `out_limit` while the
> controller keeps independent `out_min`/`out_max`. Every instance the cascade creates today is
> symmetric, so nothing is lost — but an asymmetric loop added later would be silently
> symmetrised here.

### The all-zero read overload

A `telemetry_pid_gains_payload_t` whose six float fields are all exactly zero is a **read
request**, not a set (see `telemetry_uart.h`). That is resolved by the caller *before* reaching
`apply_gains()`; a payload with real gains and zero limits is a genuine partial set and is
handled as described above.

## Debug streaming

`pid_registry_publish()` is called at the end of every `pid_update()` in the cascade — i.e. it is
permanently in the 1 kHz path. It stays cheap because of the ordering:

1. Is streaming enabled at all? (`divider == 0` → return). **This is the entire cost on an
   idle link**, which is what makes it acceptable to leave in permanently.
2. Is this loop subscribed? (`selected != PID_LOOP_ALL && selected != id` → return)
3. Has the decimation counter come round? (`++decimator[id] < divider` → return)
4. Build the 30-byte sample and zero-timeout-send it.

Nothing streams until the ground station asks. Starting subscribed to every loop at 1 kHz would
fill the queue before anyone was listening.

### Queue sizing

64 samples × 30 bytes is under 2 KB and covers ~128 ms at 500 Hz — far more slack than the
telemetry task's 20 ms cycle needs. Deeper would only buy the ability to fall further behind
before dropping, which is not a useful thing to buy.

`pid_registry_get_drop_count()` is the health signal: non-zero and rising means the telemetry task
is not draining fast enough for the selected divider, so the plot has gaps. Raise the divider.

### Flags

`PID_FLAG_I_CLAMPED` and `PID_FLAG_OUT_SAT` come straight from the `pid_controller_t` telemetry
fields. **`PID_FLAG_MEAS_STALE` has no producer** on this airframe — nothing tracks per-loop
measurement age, and the loops that could go stale (altitude, velocity) simply stop running when
their nav validity flag drops rather than running on an old sample.

## Test-signal injection

`pid_registry_inject_offset(id, t_s)` returns a value to **add to a loop's setpoint** before
calling `pid_update()`. Only one loop is injected at a time, so `set_inject()` replaces rather
than adds. `t = 0` is latched inside `set_inject()` so the ground station never has to know
anything about the flight controller's clock.

| Mode | Signal | Use |
|---:|---|---|
| 0 | off | cancel |
| 1 | **step** — constant from the trigger onwards | The ground station's overshoot and settling-time readouts are computed from this one. |
| 2 | **doublet** — `+A` for half a period, `-A` for half, then nothing | Excites the loop in both directions and **leaves the aircraft where it started**, which is why this is the safe one to use in the air. |
| 3 | **square** — continuous alternation | Watching the loop settle repeatedly. |

## Public API

| Function | Called from | Blocking? |
|---|---|---|
| `pid_registry_init()` | `app_main`, before `flight_control_init()` | startup |
| `pid_registry_bind(id, pid)` | flight controller init, once per loop | no |
| `pid_registry_apply_gains(&g)` | telemetry task | mutex, 10 ms timeout |
| `pid_registry_read_gains(id, &out)` | telemetry task | mutex, 10 ms timeout |
| `pid_registry_set_stream(loop, div)` | telemetry task | no |
| `pid_registry_set_inject(&inj)` | telemetry task | no |
| `pid_registry_inject_offset(id, t)` | **`fc_task`** | **never** |
| `pid_registry_publish(id, pid)` | **`fc_task`** | **never** (drops instead) |
| `pid_registry_pop_debug(&out, wait)` | telemetry task | caller's choice |
| `pid_registry_get_drop_count()` | anywhere | no |

The registry **does not own** the controllers and never frees them — they are statics inside
`flight_control` that outlive it.

### Degraded modes

If `xQueueCreate` fails, streaming is disabled and everything else still works. If the mutex
fails to create, live tuning is disabled and the flight loop is unaffected. Both are logged at
init and neither is fatal — instrumentation must never be able to ground the aircraft.

## Loop ids: `pid_loop_id_t`, not `telemetry_loop_id_t`

This component is addressed by `pid_loop_id_t` (ordered **outer-to-inner**: altitude, velocity,
angle, rate), which is the ground station's numbering. The web dashboard uses a *different* enum,
`telemetry_loop_id_t` (inner-to-outer), whose values are baked into `index.html`.

`pid_registry_bind()` is what reconciles them: each PID instance keeps its `telemetry_loop_id_t`
array slot in `flight_control` **and** is bound to its `pid_loop_id_t` here. See
[`shared_components/telemetry_uart/README.md`](../../../shared_components/telemetry_uart/README.md).

## Dependencies

**Public** (`REQUIRES`): `pid_controller`, `telemetry_uart`, `freertos` — all three appear in
`pid_registry.h`'s signatures. **Private** (`PRIV_REQUIRES`): `esp_timer`, used only for
timestamping samples and latching the injection `t=0`.

`flight_control` depends on this component **privately** — nothing in `flight_control.h` mentions
it, because the registry is an implementation detail of the cascade.

## Files

| File | Contents |
|---|---|
| `include/pid_registry.h` | Ten public functions, with the blocking contract documented per function. |
| `pid_registry.c` | Queue + mutex, gain apply/read, stream selection, the four injection waveforms, publish. |
| `CMakeLists.txt` | Component registration with the public/private dependency split explained. |
