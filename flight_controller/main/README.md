# `main` — startup, task layout and the telemetry link

The flight controller's entry point. This directory contains no control logic of its own — it
creates the three FreeRTOS tasks, brings the components up in the right order, and owns the
cross-task plumbing that lets a 1 kHz loop, a 100 Hz loop and a 50 Hz loop share data without any
of them blocking the others.

## The three tasks

| Task | Core | Priority | Rate | Job |
|---|---:|---:|---:|---|
| `fc_task` | 1 | `configMAX_PRIORITIES - 1` | 1000 Hz | The entire control loop: IMU read → attitude → nav snapshot → cascade → motors |
| `sensor_task` | 0 | 6 | 100 Hz | ToF (every 3rd cycle) + optical flow (every cycle) → `nav_estimator` → publish `nav_state_t` |
| `telemetry_task` | 0 | 5 | 50 Hz | Status frame, PID debug drain, IMU frame every 5th cycle, drain all inbound frames |

The split is by **timing sensitivity**, not by subject matter.

- `fc_task` gets core 1 (`APP_CPU`) because core 0 (`PRO_CPU`) runs Wi-Fi, lwIP and other ESP-IDF
  background work. Nothing slow — I²C clock stretching, SPI, UART, logging — is allowed in it once
  it is past startup.
- `sensor_task` is on core 0 specifically **because** its sensors are slow blocking I/O. Letting
  either of them into `fc_task` would cause it to miss its 1 ms deadline.
- `telemetry_task` is on core 0 for the same reason.

## Startup order in `app_main()` — and why it is this order

```
 1. imu_data_mutex, imu_ready_semaphore
 2. motor_driver_init()          <- FIRST. Motors driven to zero before anything can command thrust.
 3. attitude_init(1/1000, 0.5)
 4. flight_control_init()        <- creates the 8 PIDs, binds them to pid_registry
 5. uart_telemetry_init()        <- UART1, TX GPIO6 / RX GPIO7, 460800 baud
 6. xTaskCreatePinnedToCore(fc_task, core 1)
       └─ fc_task: imu_setup() -> imu_calibrate_acc() -> imu_calibrate_gyro()
                   -> give(imu_ready_semaphore) -> enter 1 kHz loop
 7. take(imu_ready_semaphore, 30 s)   <- app_main BLOCKS here
 8. sensor_task_init() + sensor_task_start(core 0)
 9. xTaskCreatePinnedToCore(telemetry_task, core 0)
```

**Step 2 must be first.** `motor_driver_init()` configures the LEDC channels with duty 0 before
any code path exists that could command thrust. (This does *not* protect against the gates
floating between chip reset and this line — that needs physical pull-downs; see the
`TODO(hardware)` in `motor_driver.c`.)

**Steps 7–8 are the interesting bit.** The ToF sensor attaches to the I²C bus that `imu_setup()`
*creates*, so touching it first would race the bus. Rather than calling `imu_setup()` from
`app_main`, the design lets `fc_task` own the IMU end to end and signals readiness with a binary
semaphore.

The 30-second timeout is sized deliberately: accel calibration is 500 samples at 2 ms, the gyro
sweep is 1000 samples and may retry up to three times if it detects motion. At a 1 kHz tick the
2 ms delay rounds up and the I²C read adds ~0.5 ms, so each sweep is nearer 3 s than 2 s — about
11 s worst case. 30 s leaves room without the ToF ever starting mid-calibration.

If the semaphore times out, navigation sensors simply never start and the drone flies in angle
mode. Consistent with everything else here: **no non-fatal failure grounds the aircraft.**

## `fc_task` — the 1 kHz loop

```
  imu_read_raw_data()            ← fails: motor_all_stop(), 10 ms backoff, retry
  imu_convert_raw_to_physical()
  compute dt from esp_timer      ← clamped: dt outside (0, 0.02] is replaced with 1 ms
  imu_compute_roll_pitch()
  publish latest_imu_payload     ← zero-timeout mutex; skipped if busy
  attitude_update() / attitude_get()
  sensor_task_get_nav_state()    ← zero-timeout mutex; keeps last copy if busy
  flight_control_update()        ← the cascade; writes the motors
  vTaskDelayUntil(1 ms)
```

Three things worth calling out:

**A dead IMU stops the motors.** No attitude means the drone must not be flying, so a failed read
calls `motor_all_stop()` and backs off rather than continuing on stale data.

**`dt` is clamped.** A scheduling hiccup producing a huge `dt` would give the cascade a huge
derivative and integrator step. Anything outside `(0, 0.02]` is replaced with the nominal 1 ms.

**Every cross-task read uses a zero timeout.** `fc_task` never blocks on `sensor_task` or
`telemetry_task`. A `nav_state_t` copy up to one 100 Hz cycle stale, or an IMU telemetry frame
that skips one publish, is always preferred over a missed 1 ms deadline.

> There is a comment in `fc_task` worth knowing about: the loop period used to be
> `pdMS_TO_TICKS(1000)` — 1000 ms — so the "1 kHz" loop was actually running at 1 Hz. It is
> `pdMS_TO_TICKS(1)` now, and that only works because `CONFIG_FREERTOS_HZ` is 1000 in
> `sdkconfig.defaults`. If the tick rate ever drops back to the IDF default of 100 Hz, this loop
> silently becomes a 100 Hz loop.

## `sensor_task` — the 100 Hz nav loop (`sensor_task.c`)

Reads attitude (for tilt and rotation compensation), polls the two nav sensors, feeds
`nav_estimator`, and publishes the result behind `nav_state_mutex`.

**ToF every 3rd cycle (~33 Hz).** The sensor is configured for a 25 ms inter-measurement period
(~40 Hz), so polling every 100 Hz cycle would mostly return "not ready" — wasted I²C transactions
on a bus the 1 kHz IMU loop is also using.

**Flow every cycle (100 Hz).** No internal rate limit on that sensor.

**A failed read passes `NULL`, not nothing.** Both `nav_estimator_update_*` calls happen either
way. Passing `NULL` is how the estimator is told "no data this cycle" so it can run its own 200 ms
timeout and eventually drop the validity flag — rather than silently freezing the last good
altitude forever.

**Neither sensor is required.** `sensor_task_init()` logs a warning and carries on if either
`vl53l1x_init()` or `pmw3901_init()` fails. `sensor_task_init()` only returns an error if the
*mutex* could not be created.

### The `nav_state_t` handoff

Two different timeouts on the same mutex, and the asymmetry is the design:

| Side | Call | Timeout | On failure |
|---|---|---|---|
| Writer (`sensor_task`, 100 Hz) | `xSemaphoreTake(..., 2 ms)` | 2 ms | Skip this publish |
| Reader (`fc_task`, 1 kHz) | `sensor_task_get_nav_state()` | **0** | Keep last copy, return false |

The reader must never wait. The writer can afford 2 ms out of its 10 ms budget.

## `telemetry_task` — the 50 Hz link

**Transmit order matters.** Status frame first, then PID debug samples, then the IMU frame:

1. **Status, every cycle.** This is what the dashboard reads and what the *telemetry module's*
   view of the link depends on, so it goes out first regardless of how much tuning traffic is
   queued behind it.
2. **PID debug, up to `PID_DEBUG_PER_CYCLE` (32) per cycle.** 32 × 50 Hz = 1600 samples/s,
   comfortably above the 500 Hz the ground station asks for at its fastest setting, so the queue
   only backs up if the UART itself is the bottleneck. The drain is zero-wait and bounded — an
   empty queue ends the loop immediately, which is the normal case. Send failures here are
   deliberately **unlogged**: this runs up to 32 times per cycle and a per-sample log line would
   saturate the console UART on its own.
3. **IMU frame, every 5th cycle (10 Hz).** Not needed for control; kept for logging and the raw
   data view.

**Receive drains everything queued**, not one frame per cycle — otherwise the RX buffer backs up
and the control frames actually acted on get progressively staler. It uses
`uart_telemetry_read_message_resync()`, which discards one bad frame instead of flushing the whole
buffer; a flush would drop good control frames and trip the flight controller's own 300 ms link
watchdog.

### Inbound message handling

| Message | Action |
|---|---|
| `TELEMETRY_MSG_CONTROL` | `flight_control_set_control_input()` — feeds the controller **and** pets the link watchdog |
| `TELEMETRY_MSG_GAINS` | `flight_control_set_gains()` (dashboard path — preserves integrators) |
| `TELEMETRY_MSG_PID_GAINS` | All six floats zero → read request, answer with `pid_registry_read_gains()`. Otherwise `pid_registry_apply_gains()` (ground-station path — resets the integrator) |
| `TELEMETRY_MSG_PID_SELECT` | `pid_registry_set_stream()` |
| `TELEMETRY_MSG_PID_INJECT` | `pid_registry_set_inject()` |

Every handler length-checks `payload_len` against the struct before casting the payload. That is
the only thing standing between a truncated frame and a read past the end of the payload buffer.

## Hardware configuration set here

| Item | Value | Status |
|---|---|---|
| Telemetry UART | `UART_NUM_1`, 460800 baud | |
| TX / RX | GPIO 6 / GPIO 7 | **`TODO(pins)`** — unconfirmed |
| Attitude `tau` | 0.5 s | Starting value |
| `fc_task` stack | 4096 | |
| `telemetry_task` stack | 4096 | |

> **Pin conflict to be aware of:** the telemetry module's side of this link is currently
> configured for GPIO 43/44 in `telemetry_module/components/uart_link/include/uart_link.h`. Both
> ends are marked TODO and neither has been confirmed against the harness. The pins do not have to
> match numerically (they are different boards), but both must match the actual wiring.

## Timing and concurrency summary

| Shared state | Writer | Reader | Mechanism |
|---|---|---|---|
| `latest_imu_payload` | `fc_task` (1 kHz) | `telemetry_task` (10 Hz) | `imu_data_mutex` — writer 0 timeout, reader 1 ms |
| `shared_nav_state` | `sensor_task` (100 Hz) | `fc_task` (1 kHz) | `nav_state_mutex` — writer 2 ms, reader **0** |
| `control_input` | `telemetry_task`, core 0 | `fc_task`, core 1 | `portMUX` spinlock, inside `flight_control` |
| PID debug samples | `fc_task` | `telemetry_task` | FreeRTOS queue, zero-timeout send that drops |
| `imu_ready_semaphore` | `fc_task` | `app_main` | Binary semaphore, 30 s timeout |

The rule throughout: **the 1 kHz side never waits.** Every mechanism above is arranged so the
worst case for `fc_task` is stale data, never a blocked tick.

## Files

| File | Contents |
|---|---|
| `main.c` | `app_main()` startup ordering, `fc_task`, `telemetry_task`, the IMU telemetry mutex. |
| `sensor_task.c` | The 100 Hz nav task, sensor init (non-fatal), the `nav_state_t` publish/copy pair. |
| `sensor_task.h` | Three public functions and the "neither sensor is required" contract. |
| `CMakeLists.txt` | Component registration — `main` depends on every other component. |

## See also

Component READMEs: [`imu_driver`](../components/imu_driver/README.md) ·
[`attitude_estimator`](../components/attitude_estimator/README.md) ·
[`nav_estimator`](../components/nav_estimator/README.md) ·
[`flight_control`](../components/flight_control/README.md) ·
[`pid_registry`](../components/pid_registry/README.md) ·
[`motor_driver`](../components/motor_driver/README.md) ·
[`vl53l1x_driver`](../components/vl53l1x_driver/README.md) ·
[`pmw3901_driver`](../components/pmw3901_driver/README.md) ·
[`telemetry_uart`](../../shared_components/telemetry_uart/README.md)
