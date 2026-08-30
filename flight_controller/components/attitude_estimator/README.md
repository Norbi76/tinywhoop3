# `attitude_estimator` — complementary filter (roll / pitch / yaw)

Turns raw IMU samples into an attitude estimate: roll and pitch fused from accelerometer + gyro,
yaw integrated from the gyro alone. This is the innermost feedback signal in the whole aircraft —
`flight_control`'s rate loop reads `roll_rate/pitch_rate/yaw_rate` from here at 1 kHz and its angle
loop reads `roll/pitch` at 250 Hz.

Small component, ~140 lines, but the three things it decides — **axis mapping, sign convention,
and when the estimate is trustworthy** — are load-bearing for everything downstream.

## The fusion problem

| Sensor | Good at | Bad at |
|---|---|---|
| Accelerometer | Absolute reference — gravity says which way is down, no drift, ever | Only true when the drone isn't accelerating. Prop wash and manoeuvres make it garbage. Noisy. |
| Gyroscope | Smooth, fast, immune to linear acceleration | Integrating rate to angle accumulates bias. Drifts without bound. |

A complementary filter takes the high-frequency part from the gyro and the low-frequency part from
the accelerometer:

```
alpha = tau / (tau + dt)

roll  = alpha * (roll_prev  + roll_rate  * dt)  +  (1 - alpha) * roll_from_accel
pitch = alpha * (pitch_prev + pitch_rate * dt)  +  (1 - alpha) * pitch_from_accel
```

With `tau = 0.5 s` at 1 kHz, `alpha ≈ 0.998`. The gyro carries essentially all of the short-term
response; the accelerometer only pulls the long-term bias out over roughly half a second. Raising
`tau` trusts the gyro more (smoother, drifts more); lowering it trusts the accelerometer more
(more responsive to real tilt, but also to vibration).

`tau` is passed into `attitude_init()`, so it is tunable from `main.c` without touching this
component.

## Yaw has no absolute reference

**Yaw is pure gyro integration.** There is no magnetometer on this airframe, so nothing corrects
it. Yaw drift is unbounded — it will wander degrees per minute and never come back.

This is why yaw is flown as a **rate** everywhere in the cascade, never as an angle: there is no
angle PID for yaw, and `telemetry_control_payload_t.yaw_setpoint` is a rate in °/s.
`attitude_state_t.yaw` exists only so the dashboard can show a heading number; it is wrapped to
[-180, 180] and is otherwise not used by the controller.

**Do not build anything that assumes absolute heading.**

## The accelerometer low-pass and gate

The three accelerometer components are low-passed at **15 Hz** (first-order RC, alpha recomputed
from the measured `dt` each call), and both the gate and the angle calculation use the filtered
signal:

```
0.7 g  <=  |a_filtered|  <=  1.3 g
```

If the drone is genuinely stationary or in steady flight, gravity dominates and `|a| ≈ 1 g`.
During an aggressive manoeuvre `|a|` departs from 1 g and the derived angle is meaningless.
Outside the window the accelerometer is **gated out** for that sample: the filter coasts on pure
gyro integration and sets `accel_valid = false`.

### ⚠ Why the low-pass exists — it fixed a real drift bug (2026-08-29)

The gate used to run on the **raw** magnitude at 1 kHz, and the window was 0.8–1.2 g. Prop
vibration swings the instantaneous magnitude far past ±0.2 g, so most samples in powered flight
failed the gate, the filter degenerated into pure gyro integration, and gyro bias accumulated as
roll/pitch error over minutes. The pilot symptom was "after a few minutes the IMU accumulates a
lot of error on roll and pitch".

The 2026-08-27 flight log isolates it cleanly: **with motors off, drift was 0.0–0.3 °/min** — the
filter was never the problem. Drift appeared only under power. The accelerometer was not wrong, it
was being discarded.

Vibration is high frequency, gravity is DC, so the two are separable in frequency. Filtering
removes the interference; widening the gate alone would only have admitted the vibration into the
angle calculation, trading drift for noise. Simulated against 1 kHz sampling with tonal prop
vibration plus broadband noise:

| Vibration | Old gate accepts | LPF + new gate accepts | Mean per-sample roll error, raw → filtered |
|---|---|---|---|
| 0.2 g (mild) | 75.5% | 100% | 4.2° → 0.5° |
| 0.5 g (typical) | 35.7% | 99.9% | 13.4° → 1.3° |
| 1.0 g (harsh) | 25.0% | 99.0% | 30.7° → 2.7° |

The added phase lag is irrelevant here: the complementary filter already weights the accelerometer
at `(1 - alpha) = dt/(tau+dt)` ≈ 0.2% per sample, so it is only ever a long-term levelling
reference. Tens of milliseconds of lag against a 0.5 s time constant does not show up in the
output.

`ACCEL_LPF_CUTOFF_HZ = 15.0f` is a sensible starting point for a whoop, **not a measured value**.
If drift persists, lower it before touching the gate.

### Watching it work

`accel_valid` is **not** carried in the 50 Hz status frame, so neither the dashboard nor the
flight logs can show it — which is why this bug stayed invisible for so long. `attitude_update()`
therefore prints the accept ratio to the USB serial console once per second
(`ATTITUDE_HEALTH_LOG_ENABLED`). Spin the props on the bench and watch it: a healthy filter accepts
nearly everything. A low ratio means the cutoff is still too high.

The gate cannot catch a *sustained coordinated* acceleration where the magnitude stays near 1 g
while the direction is wrong — but that needs a precise bank angle held for seconds, which a
hand-flown whoop does not do.

## Initialisation gate

`attitude_state_t.initialized` becomes true only after **500 consecutive good accelerometer
samples** (0.5 s at 1 kHz of the drone sitting reasonably still). A gated-out sample does not
count toward the total, though it also does not reset it.

Two things happen during that window:

1. Roll and pitch are **seeded straight from the accelerometer** every sample rather than fused.
   Without this the filter would creep in from zero over its own time constant, and the first half
   second of flight would be spent fighting a bogus initial angle.
2. `attitude_is_initialized()` returns false, and **`flight_control` refuses to arm**. The state
   is also reported to the dashboard as `TELEMETRY_STATUS_FLAG_ATTITUDE_INIT`.

## ⚠ Axis mapping and signs are NOT verified

This is the most important unverified thing in the flight controller, and it is flagged in three
places (`attitude_estimator.h` field comments, a block comment in `attitude_update()`, and
`flight_controller/CLAUDE.md`).

The code assumes the IMU is mounted **X forward, Y right, Z up**, with the sensor's own axes not
rotated relative to the airframe:

```c
state.roll_rate  = imu->gyro_x_dps;
state.pitch_rate = imu->gyro_y_dps;
state.yaw_rate   = imu->gyro_z_dps;
```

That is an assumption about how the IMU sits on **your** PCB, and it is wrong about as often as
it is right.

### Bench check (props off, drone disarmed, watch the status telemetry)

| Move the airframe | Expected |
|---|---|
| Roll **right** | `roll_rate` goes **positive** |
| Pitch **nose up** | `pitch_rate` goes **positive** |
| Yaw **nose right** | `yaw_rate` goes **positive** |
| Hold right side down | `roll` goes **positive** |
| Hold nose up | `pitch` goes **positive** |

If an axis reads backwards, negate it in `attitude_update()`. If two axes are swapped, swap them
there. **Fix it here and it is fixed for the entire cascade** — do not compensate downstream in
`flight_control`, or the two corrections will fight each other later.

## Public API

| Function | Notes |
|---|---|
| `attitude_init(nominal_dt, tau)` | Clears state, stores `tau`. `nominal_dt` is validated but the actual per-call `dt` is what's used. Rejects non-positive arguments. |
| `attitude_update(&imu, dt)` | One filter iteration. Silently returns on `NULL` or non-positive `dt` — a skipped update is better than a corrupted state. |
| `attitude_get(&out)` | Struct copy of the current estimate. |
| `attitude_is_initialized()` | The arming precondition. |
| `attitude_reset()` | Clears state and forces re-init. Preserves `tau`. |

## Timing and concurrency

- `attitude_update()` runs in `fc_task` at 1 kHz, core 1, immediately after the IMU read. It is
  pure float arithmetic plus one `sqrtf` — no I/O, no allocation, no blocking.
- `dt` is passed per call rather than assumed, so jitter in the loop is absorbed correctly rather
  than integrated as an error.
- All state is module statics with **no locking**. Safe because `fc_task` is the only writer *and*
  the only meaningful reader. `telemetry_task` also calls `attitude_get()` for the status frame;
  that copy is not atomic, so a status frame can in principle mix fields from two consecutive
  1 kHz ticks. For a 50 Hz display of a slowly-changing quantity that is irrelevant — but it would
  not be acceptable if anything in a control path ever read it cross-task.

## Unverified assumptions and TODOs

| Marker | Location | What must be checked |
|---|---|---|
| `SIGN NEEDS BENCH VERIFICATION` | `attitude_estimator.h` (roll, pitch) | Angle sign convention |
| `AXIS MAPPING NEEDS BENCH VERIFICATION` | `attitude_estimator.h` (all three rates) | Gyro axis → body axis assignment |
| Block comment | `attitude_update()` | The full bench procedure, reproduced above |

The LPF cutoff (15 Hz), the gate window (0.7–1.3 g), `tau` (0.5 s) and the init sample count (500)
are reasonable starting values, not measured ones — but unlike the axis mapping, getting them
slightly wrong degrades performance rather than inverting a control loop.

That said, the cutoff is the one to watch: it is the constant that decides whether the
accelerometer participates in the fusion at all under power. The old raw-signal gate is proof that
getting this wrong is not a mild degradation — it silently turned a fused filter into a bare
integrator. Check the accept ratio in the serial log after any change to props, motors or frame.

## Dependencies

`imu_driver` (for `imu_physical_data_t` and `imu_compute_roll_pitch()`). Nothing else — no
FreeRTOS, no ESP-IDF drivers, no timers.

Consumed by `flight_control` (rates at 1 kHz, angles at 250 Hz), `sensor_task`/`nav_estimator`
(tilt compensation), and `telemetry_task` (status frame).

## Files

| File | Contents |
|---|---|
| `include/attitude_estimator.h` | `attitude_state_t` with its sign conventions, five public functions. |
| `attitude_estimator.c` | Gate constants, the filter, the axis-mapping bench procedure. |
| `CMakeLists.txt` | Component registration. |
