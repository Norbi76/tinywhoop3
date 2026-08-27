# `imu_driver` — MPU-9250-class 6-axis IMU over I²C

The lowest layer of the attitude pipeline. It brings up the I²C bus, configures the IMU's ranges
and low-pass filters, reads the raw 16-bit sensor registers, converts them to physical units, and
removes the sensor's zero-rate bias with a boot-time calibration.

It does **not** do any fusion — no filter state, no angle tracking over time. That belongs to
[`attitude_estimator`](../attitude_estimator/README.md). This component's only "smart" output is
`imu_compute_roll_pitch()`, which is a stateless trigonometric read of where gravity is pointing
in this one sample.

## Hardware

| Item | Value |
|---|---|
| Bus | `I2C_NUM_0`, 400 kHz, internal pull-ups enabled |
| SDA / SCL | GPIO 12 / GPIO 11 (`I2C_MASTER_SDA` / `I2C_MASTER_SCL` in `imu_driver.c`) |
| Address | `0x68` |
| `WHO_AM_I` expected | **`0x74`** |

`imu_setup()` is what **creates** the I²C master bus. The ToF rangefinder
([`vl53l1x_driver`](../vl53l1x_driver/README.md)) sits on the same bus, which is why
`app_main()` waits on `imu_ready_semaphore` before starting `sensor_task` — touching the ToF
before this function has run would race the bus creation.

### About that `WHO_AM_I`

A genuine InvenSense MPU-9250 reports `0x71`. This board's part reports `0x74`, i.e. it is a
clone. That is recorded here rather than "fixed" because it has a real consequence: the noise
floor is several times the datasheet figure, which is why the calibration motion threshold below
is set as loosely as it is.

## Configuration applied at setup

| Register | Value | Meaning |
|---|---|---|
| `PWR_MGMT_1` (0x6B) | `0x80` then `0x01` | Device reset, 100 ms wait, then auto-select best clock source |
| `CONFIG` (0x1A) | `0x03` | **Gyro** DLPF → 41 Hz |
| `GYRO_CONFIG` (0x1B) | `0x18` | ±2000 °/s |
| `ACCEL_CONFIG` (0x1C) | `0x10` | ±8 g |
| `ACCEL_CONFIG2` (0x1D) | `0x03` | **Accel** DLPF → 41 Hz |

The last line matters more than it looks. `CONFIG` filters *only* the gyroscope; the accelerometer
has its own DLPF in `ACCEL_CONFIG2`, whose post-reset default is 460 Hz. Without that write the
accelerometer ran effectively unfiltered while the gyro ran at 41 Hz — and prop vibration went
straight into the accelerometer. Since the complementary filter treats the accelerometer as its
answer to "which way is down", that is vibration being read as attitude, and the drone stops
holding angle. Both sensors are now on the same 41 Hz corner (11.8 ms group delay).

Scale factors follow from the ranges and the 16-bit ADC:

```
accel:  32768 / 8 g       = 4096 LSB/g
gyro:   32768 / 2000 dps  ≈ 16.4 LSB/(°/s)
temp:   333.87 LSB/°C, +21 °C offset (datasheet)
```

## Public API

| Function | What it does |
|---|---|
| `imu_setup()` | Creates the I²C bus, adds the device, resets and configures the IMU, verifies `WHO_AM_I`. Idempotent — returns `ESP_OK` immediately if already set up. |
| `imu_read_raw_data(&raw)` | One 14-byte burst read from `ACCEL_XOUT_H`, unpacked into seven `int16_t`. |
| `imu_convert_raw_to_physical(&raw, &phys)` | Applies scale factors and subtracts the gyro bias. |
| `imu_calibrate_gyro()` | Measures and stores the gyro zero-rate bias. See below. |
| `imu_calibrate_acc()` | Measures and stores a roll/pitch mounting offset. |
| `imu_compute_roll_pitch(ax, ay, az, &roll, &pitch)` | Stateless gravity-vector angles, mounting offset removed. |

All six sensor axes plus temperature arrive in a **single** 14-byte I²C transaction
(`ACCEL_XOUT_H` through `GYRO_ZOUT_L` are contiguous). At the 1 kHz loop rate that single-burst
property is not an optimisation detail — three separate transactions would not fit in the budget,
and would also sample the axes at slightly different instants.

## Calibration

### Gyro bias (`imu_calibrate_gyro`)

1000 samples at 2 ms = **2 seconds** of averaging, up to 3 attempts.

The interesting part is the **motion check**. Averaging a window during which the drone was moved
produces a plausible-looking but wrong offset, and a wrong Z offset is indistinguishable in flight
from a drone that slowly rotates on its own — the controller subtracts the bad bias and concludes
the yaw rate is zero. So the routine tracks peak-to-peak spread per axis and rejects the window if
the worst axis exceeds `GYRO_CALIB_MAX_SPREAD_LSB` (200 LSB ≈ 12 °/s).

That threshold is deliberately loose, for the clone-noise reason above; a tight threshold would
false-alarm every boot, which is worse than the check that used to be missing entirely. The
success log prints the measured worst spread — there is a `TODO(bench)` in the source asking you
to watch that number across a few boots on your hardware and tighten the limit to about 3× typical.

If all three attempts see motion, the function **logs an error and returns anyway** rather than
blocking boot. Yaw will drift and roll/pitch will lean, but you get a bootable drone and a loud
message instead of a hang.

### Accel mounting offset (`imu_calibrate_acc`)

500 samples, averaged. Computes the roll/pitch the gravity vector reports while the drone is
sitting level, and stores it as `roll_offset` / `pitch_offset`. This absorbs the IMU not being
mounted perfectly square to the frame. Unlike the gyro routine it has **no motion check** and
cannot fail — a bad accel calibration shows up as a persistent lean, which is far more obvious to
the pilot than a yaw bias.

## Timing and concurrency

- Everything here is **blocking** I²C with an infinite timeout (`-1`). Both calibration routines
  additionally `vTaskDelay()` for seconds at a time.
- Consequence: `imu_setup()`, `imu_calibrate_gyro()` and `imu_calibrate_acc()` are **startup-only**
  calls. They run inside `fc_task` before it enters its 1 kHz loop.
- `imu_read_raw_data()` is the only function called in the hot loop, and it is one 400 kHz burst
  of 14 bytes plus the register address — roughly 375 µs of bus time, comfortably inside the 1 ms
  tick.
- The offsets (`gyro_offset_*`, `roll_offset`, `pitch_offset`) and the bus/device handles are
  module-level statics with no locking. Safe only because a single task owns the IMU. Nothing else
  in the firmware calls into this component.

## Unverified assumptions and TODOs

- `TODO(bench)` in `imu_calibrate_gyro()` — tighten `GYRO_CALIB_MAX_SPREAD_LSB` once you have real
  worst-spread numbers from your board.
- Axis sign and mapping conventions are **not** decided here — this component reports what the
  chip reports. The conventions, and their bench-verification markers, live in
  [`attitude_estimator`](../attitude_estimator/README.md).

## Dependencies

`esp_driver_i2c` only. Nothing in this repo other than `attitude_estimator` and `fc_task` calls it.

## Files

| File | Contents |
|---|---|
| `include/imu_driver.h` | `imu_raw_data_t`, `imu_physical_data_t`, the six public functions. |
| `imu_driver.c` | Register map, bus/device setup, burst read, unit conversion, both calibrations. |
| `CMakeLists.txt` | Component registration. |

> Some comments in the source are in Romanian. That is the primary author's language and is
> intentional existing style — add alongside it, don't translate it.
