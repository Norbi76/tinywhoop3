# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

The flight controller firmware for a custom tiny-whoop quadcopter, running on an ESP32-S3 (XIAO
ESP32-S3 board) under ESP-IDF. It runs a 1 kHz sensor-fusion + cascaded-PID control loop and talks
to a companion project, `../telemetry_module` (a separate ESP32 board running the pilot's UART
link, wifi AP and web dashboard), over a point-to-point UART protocol defined in
`../shared_components/telemetry_uart`. That shared component is pulled in via
`EXTRA_COMPONENT_DIRS` in the top-level `CMakeLists.txt` — it is not vendored, so changes to the
wire protocol must stay compatible with (or be made alongside changes to) `telemetry_module`.

Sibling directories under `../` (`documents/`, `tinywhoop_frame/` — the KiCad frame board) are
reference material, not part of this build.

## Build / flash / monitor

This is a standard ESP-IDF project (`idf.py`), targeting `esp32s3`. There is no test suite.

```bash
idf.py build
idf.py -p /dev/ttyACM0 flash
idf.py -p /dev/ttyACM1 monitor        # separate USB port from flash in this setup, see .vscode/settings.json
idf.py -p /dev/ttyACM0 flash monitor
idf.py menuconfig
```

`sdkconfig.defaults` is the authoritative config (1000 Hz FreeRTOS tick, 240 MHz CPU, perf-optimised
build, custom 4 MB partition table). `sdkconfig`/`sdkconfig.old` are generated and can be deleted to
regenerate from defaults if they drift.

## Architecture

### Task layout (`main/main.c`, `main/sensor_task.c`)

Three FreeRTOS tasks, deliberately split by timing sensitivity:

- **`fc_task`** (core 1, highest priority, 1 kHz) — the entire control loop: IMU read →
  `attitude_update()` → pulls the latest nav state → `flight_control_update()` (writes motors).
  Nothing slow (I2C clock stretching, SPI) is allowed in this task once it's past startup.
- **`sensor_task`** (core 0, 100 Hz, `main/sensor_task.c`) — polls the ToF (every 3rd cycle,
  ~33 Hz, matched to the sensor's own inter-measurement period) and the optical flow sensor (every
  cycle), feeds `nav_estimator`, and publishes a mutex-protected `nav_state_t`. Kept off core 1 and
  off the 1 kHz loop specifically because both sensors are slow blocking I/O.
- **`telemetry_task`** (core 0, 50 Hz) — sends a status frame every cycle, an IMU frame every 5th
  cycle, and drains all pending inbound frames (control setpoints, gain updates) using the
  resync reader so one bad byte doesn't cost the whole RX buffer.

Cross-task handoff is always a mutex-protected copy with a short/zero timeout on the 1 kHz side —
`fc_task` must never block on `sensor_task` or `telemetry_task`. A stale copy (at most one 1 kHz
tick, or one 100 Hz cycle, old) is preferred over blocking.

Startup ordering in `app_main()` matters: motors are initialised first (driven to zero before
anything can command thrust), then `fc_task` is started and does IMU setup + calibration, then
`app_main` waits on `imu_ready_semaphore` before starting `sensor_task` — because the ToF shares
the I2C bus that `imu_setup()` creates, and touching it first would race the bus.

### Control cascade (`components/flight_control`)

Three nested PID loops, all phase-locked to the 1 kHz `fc_task` tick via divider counters (not
separate tasks):

- **Inner, 1000 Hz** (every tick): rate → mixer → motors. Only needs the IMU.
- **Mid, 250 Hz** (every 4 ticks): angle → rate setpoints. Only needs the IMU.
- **Outer, 50 Hz** (every 20 ticks): altitude → throttle, velocity → angle setpoints. The only
  layer that depends on `nav_estimator`, and it self-disengages the instant `altitude_valid` /
  `velocity_valid` goes false — so the drone is always flyable in angle mode with no ToF/flow
  fitted, and outer-loop hardware failures degrade gracefully rather than faulting.

8 `pid_controller_t` instances in total (3 rate + 2 angle + 2 velocity + 1 altitude), addressed by
`telemetry_loop_id_t` for live gain tuning from the dashboard. `flight_control` also owns the
arming state machine (`FLIGHT_STATE_DISARMED` / `ARMED` / `KILLED`) and the control-link watchdog
(disarms after 300 ms without a valid control frame).

### Sensor fusion

- **`attitude_estimator`**: complementary filter (accel + gyro → roll/pitch). Yaw is gyro
  integration only — there is no magnetometer on this airframe, so yaw drift is unbounded and yaw
  is always flown as a rate, never as an absolute angle. Don't build anything that assumes
  absolute heading.
- **`nav_estimator`**: turns ToF range + optical flow into altitude/climb-rate/body-velocity, each
  with its own validity flag that can go false at any time (featureless floor, high tilt, out of
  ToF range, or hardware simply not fitted). Anything downstream must check the flag, never assume
  the value is good.

### Motor mixer (`components/motor_driver`)

4 coreless brushed motors via LEDC PWM. Motor index ordering (`MOTOR_FRONT_LEFT` = 0 … `M3` = 3)
and CW/CCW spin direction are hard dependencies of the mixer's yaw sign in `flight_control` — see
the diagram in `motor_driver.h`. This is a bench-verification item, not just documentation: get it
wrong and the drone spins in yaw instead of holding heading.

### Telemetry protocol (`../shared_components/telemetry_uart`)

Framed binary protocol: `[start_byte | msg_type | payload_len | seq] [payload 0..64B] [crc16 | end_byte]`.
Message types and payload structs (`telemetry_control_payload_t`, `telemetry_status_payload_t`,
`telemetry_gains_payload_t`, `telemetry_imu_payload_t`) are the shared contract with
`telemetry_module` — `_Static_assert`s in the header enforce every payload fits in one frame.
Prefer `uart_telemetry_read_message_resync()` over the plain reader on a link with steady periodic
traffic: it discards one bad frame instead of flushing the whole RX buffer, which matters because a
flush at 50 Hz is enough sustained loss to trip the 300 ms link watchdog.

## Things that need bench verification before flight

Several sign/axis/direction assumptions are marked in code comments as unverified on real hardware
rather than assumed correct — search for "NEEDS BENCH VERIFICATION" and "NEEDS VERIFICATION"
(`attitude_estimator.h`, `nav_estimator.h`, `motor_driver.h`) before relying on their sign
conventions. UART pin assignments in `main.c` (`app_main`, `TX=GPIO6`/`RX=GPIO7`) are also flagged
TODO pending wiring confirmation.

## Code comments

Some source comments are in Romanian (the primary author's first language) mixed with English —
this is intentional/existing style, not something to "fix" when editing nearby code.
