# `vl53l1x_driver` — VL53L1X time-of-flight rangefinder

The downward-facing laser rangefinder. It measures distance to the surface below the drone, which
`nav_estimator` turns into altitude and climb rate, which the outer loop turns into altitude hold.

## The three-layer split

This component is **not** all project code. It has three distinct layers and it matters which is
which:

```
  vl53l1x_driver.c / include/vl53l1x_driver.h   <- OURS. Setup, config, one-call read.
        │  calls
  st_uld/VL53L1X_api.c  VL53L1X_calibration.c   <- ST's Ultra Lite Driver. VENDORED, UNMODIFIED.
        │  calls                                   STSW-IMG009 v3.5.5, BSD SLA0103.
  vl53l1_platform.c / include/vl53l1_platform.h <- OURS. Nine functions ST's code calls to
                                                    reach the I2C bus. The porting contract.
```

- **Do not edit anything in `st_uld/`.** It is third-party, has its own `LICENSE.txt` and
  `README.md`, and is compiled with `-Wno-error` because it is not warning-clean against IDF's
  settings.
- `vl53l1_platform.c` is the *only* place ST's code touches hardware. Nine functions, all
  register I/O plus a delay.

### The ULD sources are optional at build time

`CMakeLists.txt` checks for `st_uld/VL53L1X_api.c` and `st_uld/VL53L1X_calibration.c`. If they are
present it compiles them in, prepends `st_uld/` to the include path (so ST's own
`vl53l1_types.h` wins over the minimal shim in `include/` if their archive ships one), and defines
`VL53L1X_ULD_PRESENT`.

If they are **absent** the component still builds. `vl53l1x_init()` then returns
`ESP_ERR_NOT_SUPPORTED`, `vl53l1x_read()` never succeeds, `nav_estimator` marks altitude invalid,
and `flight_control` disengages the altitude and position loops. **The drone stays flyable in
angle mode.** That graceful-degradation chain is the reason the whole ToF path is optional rather
than fatal.

## Hardware

| Item | Value |
|---|---|
| Bus | `I2C_NUM_0` — **shared with the IMU**, 400 kHz |
| Address | `0x29` (does not clash with the IMU at `0x68`) |
| XSHUT | GPIO 13 (`VL53L1X_XSHUT_GPIO`), active low |

This component **does not create** the I²C bus. It calls `i2c_master_get_bus_handle(I2C_NUM_0, …)`
to claim the one `imu_setup()` already made, then adds itself as a second device. Consequence:
**`imu_setup()` must have run first**, which is why `app_main()` blocks on `imu_ready_semaphore`
before starting `sensor_task`. If it hasn't, `vl53l1x_init()` logs "did imu_setup() run first?"
and returns the error.

`XSHUT` is the active-low shutdown pin, pulsed low→high at init to bring the part up in a known
state. If your board hard-wires it to 3V3, set `VL53L1X_XSHUT_GPIO` to `-1` and the reset pulse is
skipped.

> **Note:** when picking a free GPIO, treat `motor_driver.c` as the authority on which pins the
> motors have (4, 5, 1, 2). The `TODO(pins)` block here reproduces that list for convenience and
> had drifted out of date once already. Every pin set in this project is still unconfirmed anyway.

## Ranging configuration

| Setting | Value | Why |
|---|---|---|
| Distance mode | **Short** | ~1.3 m ceiling but far better ambient-IR immunity. Correct for an indoor drone flying low over a floor — long mode reaches 4 m but is unusable under bright room lighting. |
| Timing budget | 20 ms | The minimum for short mode. |
| Inter-measurement | 25 ms → 40 Hz | Comfortably faster than the ~33 Hz at which `sensor_task` actually polls, so a fresh sample is always waiting. |
| Offset | 0 mm | **Uncalibrated** — see below. |

### Offset calibration is not done

`VL53L1X_OFFSET_MM` is `0`. The sensor has a systematic offset that depends on the cover glass in
front of it. ST's procedure: place a 17 % grey target at exactly 140 mm, run
`VL53L1X_CalibrateOffset()` once, and hardcode the returned value. Leaving it at zero typically
costs a couple of centimetres of absolute altitude accuracy — which is exactly the quantity
altitude hold is regulating. Marked `TODO(bench)`.

## Public API

| Function | Notes |
|---|---|
| `vl53l1x_init()` | Claims the bus, pulses XSHUT, waits for boot, loads ST's defaults, applies the config above, starts continuous ranging. Idempotent. **Never aborts** on missing hardware — logs and returns an error. |
| `vl53l1x_read(&out)` | Non-blocking. Returns `ESP_ERR_NOT_FINISHED` and leaves `out` untouched if no new measurement is ready. |
| `vl53l1x_is_available()` | True once init succeeded and the sensor is ranging. |
| `vl53l1_platform_set_handle(h)` | Internal plumbing — `vl53l1x_init()` hands the device handle down to the platform layer. Not for callers. |

`vl53l1x_result_t.distance_mm` is the **raw line-of-sight range, not tilt compensated**. A drone
banked at 20° over a flat floor reads about 6 % long. Tilt compensation happens in
`nav_estimator`, not here — this component reports what the sensor saw.

`range_status` is ST's status code; `0` means good. The `valid` field is just the convenience
form of `range_status == 0`. Callers must check it: a return of `ESP_OK` only means the I²C
transaction worked, not that the measurement is trustworthy.

### Clearing the interrupt is mandatory

`vl53l1x_read()` calls `VL53L1X_ClearInterrupt()` after every successful read. Skip it and the
sensor never produces another measurement — it presents as the sensor working exactly once and
then going permanently "not ready".

## The platform layer (`vl53l1_platform.c`)

Nine functions are the *entire* porting contract of the ULD. ST's code calls nothing else to
reach hardware, and the signatures must match ST's headers exactly or their `.c` files won't
compile.

Points worth knowing:

- **Register indices are 16-bit**, big-endian on the wire — unlike the IMU's 8-bit registers.
  Every write stages `[index_hi, index_lo, payload…]` into one buffer so the whole thing is a
  single I²C transaction; every read uses `i2c_master_transmit_receive()` for a repeated start
  rather than releasing the bus between the index write and the data read.
- The `dev` argument is the sensor's 7-bit address, passed opaquely by ST so one build can drive
  several sensors. We have one, so it is `(void)`-discarded and the handle is held in a
  module static set by `vl53l1_platform_set_handle()`.
- `VL53L1_MAX_TRANSFER` is 64 bytes. The largest thing the ULD actually asks for is the 17-byte
  results block; the rest is headroom.
- `VL53L1_I2C_TIMEOUT_MS` is 20 ms. It **cannot be zero** — the part clock-stretches during
  ranging — but it must stay well under `sensor_task`'s 10 ms… 30 ms budget so a dead sensor
  degrades the loop rather than stalling it.
- `VL53L1_WaitMs()` blocks the calling task. ST only calls it during init and calibration, never
  in the ranging read path, so it never blocks the 100 Hz loop.

## Timing and concurrency

- Called only from `sensor_task` (core 0, 100 Hz), and only on every third cycle (~33 Hz) to match
  the sensor's own 25 ms inter-measurement period. This is deliberately kept off core 1 and out of
  the 1 kHz loop: it is slow, blocking, clock-stretching I²C.
- `sensor_available` and `device_handle` are unlocked module statics — single-task ownership.
- The IMU (1 kHz, `fc_task`, core 1) and this sensor (33 Hz, `sensor_task`, core 0) share a
  physical bus. The IDF I²C master driver serialises transactions internally, so the risk here is
  latency, not corruption: a 20 ms clock-stretching ToF transaction could in principle delay an
  IMU read. That is why the ToF timeout is bounded, and why the ToF budget is kept at the
  short-mode minimum.

## Unverified assumptions and TODOs

| Marker | What must be checked |
|---|---|
| `TODO(pins)` | `VL53L1X_XSHUT_GPIO = 13` against actual wiring |
| `TODO(bench)` | Run ST's offset calibration and set `VL53L1X_OFFSET_MM` |

## Dependencies

`esp_driver_i2c`, `esp_driver_gpio`. Runtime ordering dependency on `imu_driver` (bus creation)
that is deliberately **not** expressed in CMake — it is enforced by `main.c`'s semaphore instead.

## Files

| File | Ours? | Contents |
|---|---|---|
| `include/vl53l1x_driver.h` | ✅ | `vl53l1x_result_t`, the four public functions. |
| `vl53l1x_driver.c` | ✅ | Bus claim, XSHUT reset, boot wait, ULD config, read + interrupt clear. |
| `include/vl53l1_platform.h` | ✅ | The nine-function ULD porting contract. |
| `vl53l1_platform.c` | ✅ | I²C implementation of those nine functions. |
| `CMakeLists.txt` | ✅ | Conditional ULD detection, `VL53L1X_ULD_PRESENT`, per-file warning suppression. |
| `st_uld/**` | ❌ ST | Vendored Ultra Lite Driver. See `st_uld/README.md` and `st_uld/LICENSE.txt`. |
