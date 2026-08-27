# `motor_driver` — four coreless brushed motors via LEDC PWM

The last stage of the control chain. `flight_control`'s mixer produces four normalised thrust
values, and this component turns them into PWM duty cycles on four MOSFET gates.

It is deliberately thin: no mixing, no arming logic, no rate limiting. It clamps, maps, writes,
and remembers what it wrote. Everything policy-shaped lives in `flight_control`.

## Motor ordering — read this before fitting props

```
            FRONT
       M0 .......... M1        M0 = MOTOR_FRONT_LEFT   spins CW
        :            :         M1 = MOTOR_FRONT_RIGHT  spins CCW
        :     +      :         M2 = MOTOR_REAR_LEFT    spins CCW
        :            :         M3 = MOTOR_REAR_RIGHT   spins CW
       M2 .......... M3
            REAR
```

Two separate things are encoded here and **both are load-bearing**:

- **The index order** is what `motor_set_thrust_all()` and `telemetry_status_payload_t.motor[4]`
  mean. Get it wrong and the dashboard shows the wrong motor, and the mixer's roll/pitch columns
  are transposed.
- **The CW/CCW assignment** sets the sign of the **yaw column** in `flight_control`'s mixer. Yaw
  authority on a quad comes from the reaction torque difference between the CW pair and the CCW
  pair. If the real prop directions do not match this diagram, a yaw command will spin the drone
  up instead of holding heading.

The CW/CCW assignment is marked `PROP DIRECTION NEEDS VERIFICATION` in `motor_driver.h`. It has
**not** been confirmed on hardware. Verify it on the bench, props off, before you trust it.

## PWM configuration

| Setting | Value | Why |
|---|---|---|
| Timer | `LEDC_TIMER_0`, low-speed mode | One timer shared by all four channels, so all four are phase-consistent |
| Channels | `LEDC_CHANNEL_0..3` | One per motor, index-aligned with the `MOTOR_*` defines |
| Frequency | 24 kHz | Above the audible-annoyance band, and well inside what a small MOSFET switches cleanly |
| Resolution | 11 bit (max duty 2047) | At 11 bits the LEDC hardware ceiling is 80 MHz / 2¹¹ ≈ 39 kHz, so 24 kHz fits with margin |

Higher resolution would force a lower carrier frequency; 11 bits at 24 kHz is the compromise
point. 2047 steps is far finer than the thrust resolution the airframe can actually use.

## Pins

| Motor | GPIO |
|---|---|
| `MOTOR_FRONT_LEFT` | 4 |
| `MOTOR_FRONT_RIGHT` | 5 |
| `MOTOR_REAR_LEFT` | 1 |
| `MOTOR_REAR_RIGHT` | 2 |

These are **placeholders** (`TODO(pins)` in the source) — confirm against your wiring before the
first powered test. Constraints when changing them: any GPIO can drive LEDC on the ESP32-S3, but
avoid the strapping pins (0, 3, 45, 46) and the USB-JTAG pins (19, 20), and GPIO 11/12 are already
the IMU's I²C bus.

## ⚠ Floating gates at boot

Between reset and `motor_driver_init()` returning, these GPIOs **float**. A floating MOSFET gate
can drift above threshold and spin a motor while you are holding the drone.

Each gate needs a **physical pull-down resistor (~10 kΩ to GND) on the PCB**. This is marked
`TODO(hardware)` in the source and is not yet verified. Software cannot fix it — the init sequence
below reduces the window but cannot eliminate it, because the window opens before any code runs.
Confirm the resistors are fitted and scope a gate during a reboot before trusting this.

The software half of the mitigation: `motor_driver_init()` is the **first** thing `app_main()`
calls, it configures every channel with `.duty = 0`, and then calls `motor_all_stop()` again
afterwards for good measure.

## Public API

| Function | Notes |
|---|---|
| `motor_driver_init()` | Configures timer + 4 channels, leaves everything at zero. Idempotent. |
| `motor_set_thrust(i, t)` | One motor, `t` in 0.0–1.0, clamped. |
| `motor_set_thrust_all(t[4])` | The hot-path call — used by the mixer every 1 kHz tick. |
| `motor_set_raw_duty(i, duty)` | Bypasses the thrust map. **Bench only** — this is how you find `HOVER_THROTTLE` and check wiring. |
| `motor_all_stop()` | Forces all four to zero. The safety path: disarm, kill, link watchdog. |
| `motor_get_last_duty(i)` | Last duty written, for telemetry and bench inspection. |

### Error handling is different in the multi-motor calls

`motor_set_thrust_all()` and `motor_all_stop()` **keep going after a failure** and return the
*first* error rather than bailing on it. A partial stop — one channel zeroed, three still holding
their previous command — is far more dangerous than a reported error. Similarly,
`motor_write_duty()` checks **both** `ledc_set_duty()` and `ledc_update_duty()`, because a silent
failure of the latter means the motor quietly keeps its old command, and during a disarm that
means it keeps spinning.

## The thrust curve

`motor_thrust_to_duty()` is currently a **straight linear map**: `duty = thrust × 2047`.

Two known limitations, both deliberate and both flagged in the source:

1. **No per-motor calibration** (`TODO(bench)`). Real coreless motors are strongly non-linear near
   the bottom of their range — they produce no usable thrust below roughly 10–15 % duty — and no
   two motors match. Proper fix: measure thrust vs duty per motor on a scale, then replace this
   with a per-motor lookup table or polynomial. Until then expect the drone to need trim.
   (`flight_control`'s `MOTOR_IDLE_THRUST` partly papers over the dead zone by keeping armed
   motors spinning above it.)
2. **No battery-voltage compensation.** This airframe has no ADC divider on the pack, so there is
   nothing to measure. As the pack sags the same command produces less thrust; the cascade's
   integrators absorb the slow part of that on their own, and the pilot absorbs the rest.

## Timing and concurrency

- Every call is a couple of register writes — no blocking, no delays. Safe in the 1 kHz loop, which
  is where `motor_set_thrust_all()` is called from.
- `driver_initialized` and `last_duty[]` are unlocked module statics. `last_duty[]` is written by
  `fc_task` and read by `telemetry_task` for the status frame; the read is a single aligned
  `uint32_t`, so it is atomic on this target and a torn value is not possible. The worst case is
  a value one tick stale, which is irrelevant for a display.
- `motor_driver_init()` must run before anything else in `app_main()`, so that motors are known
  to be at zero before any code path can command thrust.

## Unverified assumptions and TODOs

| Marker | Location | What must be checked |
|---|---|---|
| `PROP DIRECTION NEEDS VERIFICATION` | `motor_driver.h` | CW/CCW per motor — decides the mixer's yaw sign |
| `TODO(pins)` | `motor_driver.c` | GPIO 4/5/1/2 against actual wiring |
| `TODO(hardware)` | `motor_driver.c` | Gate pull-down resistors fitted, scoped during reboot |
| `TODO(bench)` | `motor_driver.c` | Per-motor thrust calibration curve |

## Dependencies

`esp_driver_ledc` only.

## Files

| File | Contents |
|---|---|
| `include/motor_driver.h` | Motor index defines, the ordering/spin diagram, the six public functions. |
| `motor_driver.c` | Pin map, LEDC configuration, thrust→duty map, duty write with both-call error checking. |
| `CMakeLists.txt` | Component registration. |
