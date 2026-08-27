# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

The flight controller firmware for a custom tiny-whoop quadcopter, running on an **ESP32-S3 Super
Mini** board under ESP-IDF. It runs a 1 kHz sensor-fusion + cascaded-PID control loop and talks
to a companion project, `../telemetry_module` (a separate ESP32 board running the pilot's UART
link, wifi AP and web dashboard), over a point-to-point UART protocol defined in
`../shared_components/telemetry_uart`. That shared component is pulled in via
`EXTRA_COMPONENT_DIRS` in the top-level `CMakeLists.txt` — it is not vendored, so changes to the
wire protocol must stay compatible with (or be made alongside changes to) `telemetry_module`.

The same UART also carries a PID tuning link to a desktop tool, `../tools/ground_station`, which
`telemetry_module` relays onto UDP without interpreting. That is what `components/pid_registry`
exists to serve.

Sibling directories under `../` (`documents/`, `tinywhoop_frame/` — the KiCad frame board) are
reference material, not part of this build. `../tools/ground_station` is project code but not part
of this build.

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
- **`telemetry_task`** (core 0, 50 Hz) — sends a status frame every cycle, then up to
  `PID_DEBUG_PER_CYCLE` (32) queued PID debug samples, then an IMU frame every 5th cycle, and drains
  all pending inbound frames (control setpoints, dashboard gain updates, and the ground station's
  `PID_*` messages) using the resync reader so one bad byte doesn't cost the whole RX buffer. Send
  order matters: the status frame goes out first regardless of how much tuning traffic is queued.

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

8 `pid_controller_t` instances in total (3 rate + 2 angle + 2 velocity + 1 altitude), stored in
`pid_loops[]` indexed by `telemetry_loop_id_t`. `flight_control` also owns the arming state machine
(`FLIGHT_STATE_DISARMED` / `ARMED` / `KILLED`) and the control-link watchdog (disarms after 300 ms
without a valid control frame).

The mixer is not a plain "throttle + mix, then clamp". A brushed motor commanded to zero has
*stopped* and produces no torque until it spins back up, so clipping low motors to zero would
destroy the attitude command exactly when it is needed. Instead `flight_control_mix()` scales the
mix to fit the available band, shifts the whole group up so the lowest motor lands on
`MOTOR_IDLE_THRUST` (bounded by `MAX_MIXER_THROTTLE_BOOST` so ALT− still descends), then shrinks
the mix if the cap bit. Don't "simplify" this back.

### Live PID tuning (`components/pid_registry`)

Sits between the cascade and the telemetry link so neither knows about the other.
`flight_control_init()` binds all 8 instances once; from then on the registry can read/write gains,
sample loop internals into a queue, and hand the control loop a test-signal offset to add to a
setpoint. Every PID call in the cascade goes through `flight_control_run_pid()`, which does the
inject-then-update-then-publish sequence.

**The rule this component exists to enforce: everything called from `fc_task` is non-blocking.**
`pid_registry_publish()` and `pid_registry_inject_offset()` take no lock, touch no UART and never
log; publishing is a zero-timeout queue send that *drops* on a full queue. The mutex guards the
gain tables against telemetry-task writers only — `fc_task` never takes it.

Two independent loop-id enums address the same 8 instances: `telemetry_loop_id_t` (inner-to-outer,
used by the web dashboard, values baked into `index.html`) and `pid_loop_id_t` (outer-to-inner, used
by the Python ground station). `pid_loop_to_telemetry[]` in `flight_control.c` is the single point
where they meet. Renumbering either silently retunes the wrong axis.

Note the two gain paths differ on purpose: `flight_control_set_gains()` (dashboard) **preserves**
the integrator for small nudges; `pid_registry_apply_gains()` (ground station) **zeroes** it,
because a whole gain set with a stale ki-scaled accumulator appears as a step in the output.

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

Message types 20–23 (`PID_DEBUG` / `PID_GAINS` / `PID_SELECT` / `PID_INJECT`) are the **tuning
link** to the Python ground station in `../tools/ground_station`. `telemetry_module` does not
interpret them; it forwards them verbatim between the UART and a UDP socket. Those four payloads
carry **exact-size** `_Static_assert`s on top of the fits-in-a-frame check, because the Python side
unpacks them with hardcoded `struct` formats — a padding byte would still build and still parse,
and would shift every float in the plot by one byte.
Prefer `uart_telemetry_read_message_resync()` over the plain reader on a link with steady periodic
traffic: it discards one bad frame instead of flushing the whole RX buffer, which matters because a
flush at 50 Hz is enough sustained loss to trip the 300 ms link watchdog.

### Optional hardware — everything degrades, nothing faults

Both navigation sensors are optional at build time *and* at runtime, and the chain is deliberate:
sensor missing → `nav_estimator` validity flag false → `flight_control` disengages that outer loop
→ **the drone stays flyable in angle mode**. `sensor_task_init()` warns and continues on either
failure. `vl53l1x_driver` even builds without ST's ULD sources present (see its `CMakeLists.txt`),
and `pmw3901_driver`'s ~80-register init sequence is a **deliberate stub** — it returns
`ESP_ERR_NOT_SUPPORTED` and the velocity loop simply never engages. Don't "fix" any of these by
making them fatal.

## Things that need bench verification before flight

Several sign/axis/direction assumptions are marked in code comments as unverified on real hardware
rather than assumed correct — search for "NEEDS BENCH VERIFICATION" and "NEEDS VERIFICATION"
(`attitude_estimator.h`, `nav_estimator.h`, `motor_driver.h`, and the mixer in `flight_control.c`)
before relying on their sign conventions. Each component's `README.md` has the step-by-step bench
procedure for its own conventions. Note that a mixer sign error and a gyro axis-mapping error look
identical from outside — verify `attitude_estimator` first, then the mixer.

Also unmeasured: `MOTOR_IDLE_THRUST` and `HOVER_THROTTLE` (`flight_control.c`),
`FLOW_COUNTS_PER_RAD` (`nav_estimator.c`), `VL53L1X_OFFSET_MM` (`vl53l1x_driver.c`), the PMW3901
SPI timing constants, and all 12 default PID gains. UART pin assignments in `main.c` (`app_main`,
`TX=GPIO6`/`RX=GPIO7`) are flagged TODO pending wiring confirmation, as are the motor gate
pull-downs in `motor_driver.c` — that last one can spin a motor in your hand and software cannot
fix it.

**When checking pins, use the right board.** This firmware runs on an **ESP32-S3 Super Mini**.
`documents/XIAO_ESP32-S3_front_pinout.png` is the *telemetry module's* board (and the spare used by
the calibration bench tools) — it is **not** the flight controller's pinout, and it is the only
pinout image in the repo. Do not validate this project's GPIO assignments against it: the XIAO
exposes only 11 GPIOs (1–9, 43, 44), so several pins used here (11, 12, 17, 18, 10) look
impossible against that image while being perfectly fine on the Super Mini.

## Per-component documentation

Every component has a `README.md` (`components/<name>/README.md`, plus `main/README.md`) covering
its public API, timing/concurrency model, hardware dependencies, and its own unverified-assumptions
table. Read the component's README before editing it; this file is the map, those are the detail.
Every source file also carries a header comment stating its purpose and mechanism.

`components/vl53l1x_driver/st_uld/` is **ST's vendored code** — do not edit it.

## Code comments

Some source comments are in Romanian (the primary author's first language) mixed with English —
this is intentional/existing style, not something to "fix" when editing nearby code.
