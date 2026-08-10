# `tof_calibration` — VL53L1X offset calibration bench tool

A throwaway ESP-IDF project that runs on a **spare ESP32-S3**, not on the drone. It exists to
produce one number: the value for `VL53L1X_OFFSET_MM` in
[`../flight_controller/components/vl53l1x_driver/vl53l1x_driver.c`](../flight_controller/components/vl53l1x_driver/vl53l1x_driver.c),
which is currently `0` and marked `TODO(bench)`.

This is a third firmware project in the repo, built separately like the other two. It is a bench
tool, not part of the aircraft.

## Why a separate project rather than doing it on the drone

- `VL53L1X_CalibrateOffset()` is a one-off ST ULD call that `vl53l1x_driver`'s public API
  deliberately does not wrap — it is a bench step, not runtime code.
- The drone's `vl53l1x_init()` borrows the I²C bus `imu_setup()` creates, so calibrating in place
  means bringing up the IMU, `sensor_task`, and the whole control stack — with four live motor
  outputs — while holding a grey card 140 mm from the sensor.

## What it reuses, and why that matters

It does **not** copy the driver. The top-level `CMakeLists.txt` points `EXTRA_COMPONENT_DIRS` at
the `vl53l1x_driver` component directory itself, so this project compiles the same source the
drone flies, including ST's vendored ULD in `st_uld/`.

That is load-bearing, not just tidiness: **the offset is only valid for the ranging configuration
it was measured under.** By calling the real `vl53l1x_init()`, calibration runs at short mode /
20 ms timing budget / 25 ms inter-measurement — exactly what flies. A hand-rolled init would give
a number that is subtly wrong in the air.

The one thing this project does differently: `main.c` creates the I²C bus itself, standing in for
the drone's `imu_setup()`.

> Note on `EXTRA_COMPONENT_DIRS`: it points at the single component directory, **not** at
> `flight_controller/components/`. Pointing at the folder would discover every component, and
> `flight_control` / `pid_registry` require `telemetry_uart` from `../shared_components`, which is
> not on this project's search path — a configure-time failure.

## What you need

| Item | Notes |
|---|---|
| Spare ESP32-S3 | Any board; pins below are configurable |
| VL53L1X breakout | The same part that goes on the drone, ideally **with the same cover glass** — see caveat below |
| 17 % grey target | ST's spec. A grey card from a photography set is the correct thing. Failing that, matte mid-grey card stock; avoid white paper (too reflective) and gloss (specular). |
| A ruler | The accuracy of the whole procedure is the accuracy of this measurement |

## Wiring

| Signal | Default GPIO | Change it in |
|---|---|---|
| SDA | 12 | `CAL_I2C_SDA_GPIO`, `main/main.c` |
| SCL | 11 | `CAL_I2C_SCL_GPIO`, `main/main.c` |
| XSHUT | **17** | `VL53L1X_XSHUT_GPIO`, `vl53l1x_driver.c` — see below |
| VIN / GND | 3V3 / GND | |

SDA/SCL default to the drone's pins for familiarity, but nothing about the offset depends on which
pins the bus runs on — change them to whatever is convenient on the spare board.

**XSHUT is the exception.** It is hardcoded to GPIO 17 in `vl53l1x_driver.c`, which this project
compiles unmodified.

These three numbers are the *drone's*, and they reach the sensor through the custom frame PCB — a
bare dev board will not necessarily break all of them out. On a XIAO ESP32-S3 in particular the
header exposes a restricted GPIO set and 17 may not be available at all. So:

- **Easiest path: tie XSHUT to 3V3.** The driver's reset pulse then lands on an unconnected
  GPIO 17 and is a harmless no-op; the sensor comes up on power-on reset instead.
- Or wire XSHUT to GPIO 17 if the board exposes it, which also exercises the drone's assumption.

**Do not leave XSHUT floating** — the part may stay in reset and `vl53l1x_init()` will report
that it never booted. (Many breakouts pull it up, but do not rely on it.)

If SDA 12 / SCL 11 are not broken out either, just change the two `#define`s — nothing about the
offset depends on the bus pins.

## Running it

```bash
idf.py set-target esp32s3
```

```bash
idf.py build
```

```bash
idf.py -p /dev/ttyACM0 flash monitor
```

(Source your ESP-IDF `export.sh` first if `idf.py` is not on your PATH.)

The program runs in four phases:

1. **Bus + init** — creates I²C, then calls the real `vl53l1x_init()`.
2. **Positioning aid** — 20 seconds of live *uncalibrated* readings, twice a second, while you set
   the target at exactly 140 mm. Set the distance with the ruler; the printed number is not
   supposed to read 140 yet — that is the entire point of calibrating.
3. **Pre-check** — averages 30 samples and warns if the result is more than 40 mm from the target
   (almost always means the ruler and `CAL_TARGET_DISTANCE_MM` disagree) or if samples are coming
   back invalid.
4. **Calibrate + verify** — runs ST's routine (it averages 50 measurements internally), prints the
   offset in a box, then restarts ranging and streams *calibrated* readings so you can check two
   known distances against the ruler.

## Then update the flight firmware

Take the printed number to `vl53l1x_driver.c:61` and replace the zero:

```c
#define VL53L1X_OFFSET_MM <the number>
```

Also worth updating at that point: the `TODO(bench)` comment block above it, and the
unverified-assumptions table in the component's `README.md`.

## Caveats worth knowing before you trust the number

- **Cover glass dominates this measurement.** The offset ST is correcting is mostly caused by
  whatever sits in front of the sensor. If the drone has a canopy, camera glass, or a printed
  mount over the ToF and the bench rig does not, the number you measure here is not the number the
  drone needs. Calibrate with the flight cover glass in place, or accept that this gets you closer
  than `0` but not all the way.
- **It corrects a constant, not a slope.** Offset calibration fixes a fixed bias. If readings drift
  proportionally with distance, that is a crosstalk problem — `VL53L1X_CalibrateXtalk()` in
  `st_uld/VL53L1X_calibration.c`, a separate procedure this tool does not run.
- **140 vs 100 mm.** `CAL_TARGET_DISTANCE_MM` is 140, matching the component README. ST's own
  `VL53L1X_calibration.h` suggests 100. Either works — the only hard requirement is that the
  `#define` equals the distance you physically measure, since ST computes
  `offset = target − average_measured`.
- Ambient IR matters. Short mode is chosen partly for this, but do not calibrate in direct
  sunlight.

## Files

| File | Contents |
|---|---|
| `CMakeLists.txt` | Pulls in `vl53l1x_driver` by pointing at the component directory |
| `sdkconfig.defaults` | Minimal — deliberately **not** a copy of the flight controller's |
| `main/main.c` | The four-phase procedure |
