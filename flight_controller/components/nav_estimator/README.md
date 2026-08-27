# `nav_estimator` — altitude and body-frame velocity from ToF + optical flow

Takes raw range (mm) from `vl53l1x_driver` and raw flow counts from `pmw3901_driver`, and produces
the four numbers the **outer** control loop needs: altitude, climb rate, forward velocity, right
velocity — each with a validity flag.

This is the only source of "where am I" on the aircraft. There is no GPS, no barometer, no
magnetometer.

## The validity flags are the point

```c
bool altitude_valid;  // ToF returning good ranges AND the drone isn't tilted too far
bool velocity_valid;  // flow surface quality adequate AND altitude_valid is true
```

**Both can go false at any moment**, in normal flight, with perfectly healthy hardware: over a
featureless floor, past 40° of tilt, above the ToF's ~1.3 m ceiling, or because the sensor simply
isn't fitted. `flight_control` re-checks these on every outer-loop iteration and disengages the
affected loop the instant one drops.

That is why the drone is always flyable in angle mode with no ToF and no flow sensor at all — and
why **nothing downstream may read `altitude` or `velocity_*` without first checking the flag**.

## Altitude path (`nav_estimator_update_range`)

Called at ~33 Hz from `sensor_task`, once per ToF measurement. Three gates then two computations:

**Gate 1 — did we get a reading the sensor liked?** `range == NULL` (failed or missing read), or
`range_status != 0`, or `!valid`.

**Gate 2 — tilt.** If `max(|roll|, |pitch|) > 40°`, bail. Past that the `cos` correction below
stops being a good model, and the ToF's narrow cone starts ranging against a wall or the furniture
rather than the floor. A wrong-but-plausible altitude is worse than no altitude.

**Tilt compensation.** The ToF measures slant range along its own boresight. With the sensor
pointing straight down through the airframe:

```
altitude = slant_range × cos(roll) × cos(pitch)
```

**Gate 3 — plausible height?** Outside 0.03 m … 1.30 m, bail. The bottom of the scale is usually
the landing gear or ground effect; the top is past what short distance mode can do.

**Climb rate.** A backward difference, `(altitude - previous) / dt`, then a first-order low-pass
with `α = 0.20`. Differentiating a noisy signal amplifies the noise, so this is smoothed harder
than velocity is — at the cost of lag, which the altitude loop's own tuning has to absorb.

### The flags don't flap

Notice that every gate above only clears `altitude_valid` **after `RANGE_TIMEOUT_US` (200 ms) of
no good reading**. A single dropped sample or a momentary bad status does not drop the flag — it
just doesn't refresh it. At 33 Hz, 200 ms tolerates about six consecutive bad samples. Without
this the flag would chatter on ordinary sensor noise and the altitude loop would engage and
disengage several times a second.

Dropping `altitude_valid` also drops `velocity_valid`, because the velocity computation needs
altitude (see below).

## Velocity path (`nav_estimator_update_flow`)

Called at ~100 Hz from `sensor_task`, every cycle.

**Gate 1 — usable flow?** `flow == NULL` or `!flow->valid` (i.e. surface quality below
`PMW3901_MIN_SQUAL`). Same 200 ms timeout behaviour.

**Gate 2 — altitude is mandatory.** Optical flow measures an **angular** rate. The same angular
rate means 10 cm/s at 20 cm altitude and 50 cm/s at 1 m. Without a trusted altitude there is no
way to convert, so the component refuses to produce a number rather than emitting a
plausible-looking wrong one.

**Counts → radians.** `flow_rad = delta_counts / FLOW_COUNTS_PER_RAD`. See the warning below.

**Gyro compensation.** The flow sensor cannot distinguish translation from rotation — pitching the
nose down scrolls the image in exactly the same way as flying forward. So the rotation the gyro
measured over the same interval is subtracted:

```
translation_x_rad = flow_x_rad - (pitch_rate × dt)
translation_y_rad = flow_y_rad - (roll_rate  × dt)
```

Note the **cross pairing**: forward/backward image motion (X) comes from *pitch*, side-to-side (Y)
from *roll*. That is geometrically correct, and it is also exactly the kind of thing that is easy
to get backwards — see the bench procedure below.

**Angular rate → linear velocity.**

```
velocity = (translation_rad / dt) × altitude
```

**Sanity limit.** Anything above 5 m/s on either axis is discarded outright, before filtering — an
indoor whoop is not doing 5 m/s, so such a sample is a glitch, and letting it into the filter would
pollute the state for several cycles. The filtered output is also clamped to the same ±5 m/s.

**Filter.** First-order low-pass, `α = 0.35`. Lighter than the climb-rate filter because velocity
is not a differentiated quantity — the sensor measures displacement directly.

## ⚠ Unverified: the flow scale factor

`FLOW_COUNTS_PER_RAD = 500.0f` is a **guess** (`TODO(bench)`). It is the single scale factor that
sets how fast the drone thinks it is moving; everything about position hold depends on it.

**How to measure it:** mount the drone at a known fixed height (say 300 mm), translate it a known
distance (say 200 mm) at a steady speed, and log the accumulated delta counts.

```
angle_moved_rad     = atan2(0.200, 0.300)
FLOW_COUNTS_PER_RAD = accumulated_counts / angle_moved_rad
```

Repeat per axis. The published figure for the PMW3901's ~42° field of view over 30×30 pixels lands
somewhere near 500 — which is where the current value comes from — but that is a sanity check, not
a substitute for measuring your own unit.

## ⚠ Unverified: velocity axis pairing and signs

The code assumes the flow sensor's X axis aligns with body X (forward), and that a positive pitch
rate produces positive `flow_x`. Both are assumptions about how the sensor is physically rotated
on **your** PCB.

### Bench check — hold the drone at fixed height over a textured surface

| Do this | Expected |
|---|---|
| Slide **forward** without tilting | `velocity_x` goes **positive** |
| Slide **right** without tilting | `velocity_y` goes **positive** |
| **Pitch** in place, no translation | `velocity_x` stays near **zero** |
| **Roll** in place, no translation | `velocity_y` stays near **zero** |

The last two are the important ones: if velocity swings when you rotate in place, the **gyro
compensation sign is wrong**. Left uncorrected, that makes the aircraft interpret its own attitude
corrections as motion, which the position loop then tries to correct — a positive feedback loop.

## Constants

| Constant | Value | Rationale |
|---|---|---|
| `FLOW_COUNTS_PER_RAD` | 500.0 | **Unmeasured.** See above. |
| `MAX_TILT_FOR_ALTITUDE_DEG` | 40 | Beyond this the `cos` model breaks and the beam finds walls |
| `MIN_VALID_ALTITUDE_M` | 0.03 | Below this it's landing gear / ground effect |
| `MAX_VALID_ALTITUDE_M` | 1.30 | ToF short-mode ceiling |
| `RANGE_TIMEOUT_US` / `FLOW_TIMEOUT_US` | 200 ms | Tolerate a few dropped samples without flapping the flag |
| `CLIMB_RATE_ALPHA` | 0.20 | Heavy — it's a differentiated quantity |
| `VELOCITY_ALPHA` | 0.35 | Lighter — measured directly |
| velocity limit | ±5 m/s | Glitch rejection, both pre-filter and post-filter |

## Public API

| Function | Notes |
|---|---|
| `nav_estimator_init(nominal_range_dt)` | Clears state. Rejects non-positive `dt`. |
| `nav_estimator_update_range(&r, roll, pitch, dt)` | Pass `NULL` for `r` to signal a failed or missing read — that's the documented way to feed the timeout. |
| `nav_estimator_update_flow(&f, roll_rate, pitch_rate, dt)` | Same `NULL` convention. |
| `nav_estimator_get(&out)` | Struct copy. |
| `nav_estimator_reset()` | Clears everything, drops both flags. |

## Timing and concurrency

- All four update/read functions are called from `sensor_task` (core 0, 100 Hz) — range every
  third cycle, flow every cycle. Kept off core 1 and out of the 1 kHz loop because the *sensors*
  are slow blocking I/O; this component itself is pure arithmetic.
- Module statics, **no locking inside this component**. The cross-task handoff is done one level
  up: `sensor_task` calls `nav_estimator_get()` into a mutex-protected `nav_state_t`, and `fc_task`
  copies that with a zero/short timeout so the 1 kHz loop can never block on the 100 Hz one. A
  copy up to one 100 Hz cycle stale is preferred over blocking. See `main/sensor_task.c`.
- `esp_timer_get_time()` is used for the staleness timeouts, so they are wall-clock and correct
  even if `sensor_task` misses cycles.

## Unverified assumptions and TODOs

| Marker | What must be done |
|---|---|
| `TODO(bench)` `FLOW_COUNTS_PER_RAD` | Measure it. Nothing about velocity means anything until you do. |
| `SIGN NEEDS BENCH VERIFICATION` (`nav_estimator.h`) | `velocity_x` / `velocity_y` sign convention |
| Block comment in `update_flow()` | Axis pairing and gyro-compensation sign — the bench procedure above |

Also relevant: `VL53L1X_OFFSET_MM` is uncalibrated in `vl53l1x_driver`, which biases altitude by a
couple of centimetres, and `PMW3901_MIN_SQUAL` is a guess, which sets when `velocity_valid` drops.

## Dependencies

`vl53l1x_driver`, `pmw3901_driver` (both public — their result structs appear in this component's
API), `esp_timer` for staleness timestamps.

Consumed by `sensor_task` (drives it) and `flight_control` (reads the published `nav_state_t`).

## Files

| File | Contents |
|---|---|
| `include/nav_estimator.h` | `nav_state_t` and its validity contract, five public functions. |
| `nav_estimator.c` | Constants, the two update paths with their gates, filters, bench procedures. |
| `CMakeLists.txt` | Component registration. |
