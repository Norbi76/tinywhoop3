# `pmw3901_driver` — PixArt PMW3901 optical flow sensor (SPI)

The downward-facing optical flow sensor. It watches the floor and reports how far the image has
shifted since the last read, which `nav_estimator` combines with the ToF altitude to produce body-
frame velocity, which the outer loop uses for position hold.

## ⚠ The init sequence is now in — and that removed a safety property

`pmw3901_init()` used to return `ESP_ERR_NOT_SUPPORTED` from a deliberate stub. The stub is gone:
PixArt's 73-write power-up sequence now lives in `pmw3901_init_registers.h`, transcribed from
Bitcraze's MIT-licensed driver (provenance and licence are in that file's header).

**This driver has not yet been run against real hardware.**

While init always failed, `velocity_valid` was permanently false and the velocity outer loop could
never engage. The project relied on that. It is now gone: with a working ToF supplying
`altitude_valid`, `flight_control` **will** engage velocity hold using `FLOW_COUNTS_PER_RAD`
(still the guess `500.0f`) and the unverified gyro-compensation axis signs. A sign error there is
positive feedback — the drone accelerates away from the hold point rather than settling on it.

**Before flying this:** measure the scale factor and check the translation signs with
`../../../flow_calibration/`, then do the rotation half of the sign check on the drone (pitch in
place → `velocity_x` must stay near zero). Do not reflash the airframe until both pass.

### Third-party content

`pmw3901_init_registers.h` contains **Bitcraze's register values, not ours** (MIT, © 2017 Bitcraze
AB). Do not "tidy" them. The only correct way to change that table is to re-transcribe it from
upstream. Everything else in this component is project code.

## Hardware

| Item | Value |
|---|---|
| Host | `SPI2_HOST` |
| SCLK / MISO / MOSI | GPIO 18 / 8 / 9 |
| CS | GPIO 10, **driven manually** |
| Mode | SPI mode 3 (CPOL=1, CPHA=1) — PixArt requirement |
| Clock | 2 MHz maximum — PixArt requirement |

**Pins confirmed against the airframe wiring, 2026-08-29.** No longer placeholders. The full
allocation, verified by reading the drivers rather than a copied list:

| GPIO | Owner | Authority |
|---|---|---|
| 1, 2, 4, 5 | Motors (RL, RR, FL, FR) | `motor_driver.c:34-37` |
| 6, 7 | Telemetry UART TX/RX | `main.c:389-390` |
| 11, 12 | I²C SCL/SDA (IMU + ToF) | `imu_driver.c:34-35` |
| 13 | VL53L1X XSHUT | `vl53l1x_driver.c:52` |
| 8, 9, 10, 18 | Optical flow SPI | this component |

### ⚠ SCLK is on an underside pad

The ESP32-S3 Super Mini only breaks out **GPIO 1–13** on its side castellations — which is why
every other peripheral above lives in that range; there were no side pins left. GPIO 18 is
soldered to a **pad on the underside of the board**.

That joint has no strain relief and is the mechanically weakest electrical connection on the
aircraft. A lifted MISO or SCLK pad presents as a floating line — a constant `0x00` or `0xFF` —
which is exactly what the dual product-ID check catches at init. **If the sensor stops
identifying after a crash, suspect the solder joint before the code.** Hot-glue the wires down
once the ID check passes.

All five (`PMW3901_SPI_HOST` plus the four GPIOs) are `#ifndef`-guarded, so a bench harness can
override them from its own build without editing this component. `../../../flow_calibration/`
does exactly that via `idf_build_set_property(COMPILE_DEFINITIONS ...)`. The drone build defines
none of them and compiles identically to before the guards existed. Note the asymmetry with the
ToF: `vl53l1x_driver` *borrows* an I²C bus so its caller picks the pins at runtime, whereas this
component *creates* its SPI bus, so the pins can only be chosen at compile time.

> **Note:** the pin block in the source lists which GPIOs are taken elsewhere. That list is a
> convenience copy and had drifted out of date once already — treat `motor_driver.c`,
> `imu_driver.c` and `main.c` as authoritative for their own pins. The list above was
> re-verified against those files on 2026-08-29.

### Why chip select is manual

`spics_io_num` is `-1` and `pmw3901_cs_low()` / `pmw3901_cs_high()` drive the line by hand.

This is not a style preference. A PMW3901 register read is *send address → wait tSRAD → clock the
data byte out*, and **CS must stay low across the whole thing**. The ESP32's hardware CS deasserts
at the end of each SPI transaction, which would split that sequence in two and break it. Hence
manual CS, with a 1 µs settle either side.

## Register access

| Direction | Address byte | Sequence |
|---|---|---|
| Write | `reg \| 0x80` (MSB set) | one 2-byte transaction, then wait tSWW |
| Read | `reg & 0x7F` (MSB clear) | address byte → wait tSRAD → data byte, CS low throughout, then wait tSRR |

### The timing constants are unverified

```
CS_SETTLE  =  50 µs   after CS falls, before the first SCLK edge
WRITE_HOLD =  50 µs   after a write's last SCLK edge, before CS rises
WRITE_GAP  = 200 µs   after CS rises following a write
tSRAD      =  50 µs   read: address byte -> data byte
READ_HOLD  = 100 µs   after a read's data byte, before CS rises
```

These now match Bitcraze's `registerWrite()` / `registerRead()` exactly — the same source the
register table came from.

**They previously did not, despite this README claiming they did.** The old values were `1 µs`
inside CS on both sides and `45 µs` after, against Bitcraze's `50 / 50 / 200`. That mattered:
partially adopting a known-working bring-up is precisely how you get a sensor that returns
confident garbage, and the init sequence is 73 consecutive writes. The delays and the register
table are now kept in lockstep deliberately.

Still `TODO(datasheet)`: these are matched to a *working implementation*, not verified against a
PixArt datasheet — there isn't one in `documents/`. Bitcraze's numbers are more conservative than
the values usually quoted for this part family, so the residual risk is wasted microseconds
rather than corruption. Violating the real minimums produces **silent data corruption**, not an
error, which is why this is worth getting right rather than tuning for speed.

Cost: a register read is ~208 µs and `pmw3901_read_motion()` does six, so ~1.25 ms per call
against `sensor_task`'s 10 ms budget. That is still comfortably inside budget, and still not a
reason to switch to the burst-read register.

## Public API

| Function | Notes |
|---|---|
| `pmw3901_init()` | GPIO + SPI setup, power-up reset, product-ID check, then the (stubbed) register sequence. Idempotent. Never aborts on missing hardware. |
| `pmw3901_read_motion(&out)` | Accumulated motion since the *previous call*. |
| `pmw3901_is_available()` | True only once the full init succeeded. |

`pmw3901_motion_t.delta_x/y` are **sensor counts, not pixels and not metres**, and they are
*deltas since the last read* — the sensor accumulates internally, so read cadence affects
magnitude. Converting counts to a physical rate needs the `FLOW_COUNTS_PER_RAD` scale factor,
which lives in `nav_estimator` and is itself an unmeasured `TODO(bench)`.

`squal` is surface quality. `valid` is set when `squal >= PMW3901_MIN_SQUAL` (40). Below that the
floor is too featureless — plain carpet, glossy laminate, a white sheet of paper — for the reading
to mean anything, and `nav_estimator` gates velocity out. **40 is a guess** (`TODO(bench)`); find
the real threshold by flying low over the surfaces you actually plan to shoot over and logging
`squal`.

**`squal` is not in the 50 Hz status frame**, so the ground station cannot plot it and the flight
logs do not contain it. To make that threshold findable at all, `nav_estimator.c` now prints
`squal` (with the deltas, the raw and filtered velocities, and *which gate* dropped
`velocity_valid`) to the USB serial console at 2 Hz — see `NAV_FLOW_LOG_ENABLED` there. Turn it
off once the sensor is characterised, or promote `squal` into the status payload if you want it
in the flight logs instead.

### Identity check uses both IDs

`PRODUCT_ID` must be `0x49` **and** `INVERSE_PRODUCT_ID` must be `0xB6`. Checking both catches a
floating MISO line, which would otherwise read as a consistent `0x00` or `0xFF` on one register and
could pass a single-ID check by accident.

## Timing and concurrency

- Called only from `sensor_task` (core 0), **every** 100 Hz cycle — unlike the ToF, which is polled
  every third cycle.
- `pmw3901_read_motion()` does **six individual register reads**, not the burst-read register.
  That is slower (six × ~55 µs of delays ≈ 350 µs), but the burst buffer layout is one more thing
  to get wrong and at 100 Hz there is time to spare. If you ever need the cycles back, the burst
  register is the first place to look.
- All the inter-transaction waits are `esp_rom_delay_us()` — **busy-waits**, not `vTaskDelay()`.
  At tens of microseconds a scheduler round trip would cost more than the delay. This is another
  reason this component must stay on `sensor_task` and off the 1 kHz loop.
- `sensor_available` and `spi_handle` are unlocked module statics; single-task ownership.
- `spi_bus_initialize()` returning `ESP_ERR_INVALID_STATE` is treated as success — it just means
  something else already brought the bus up.

## Unverified assumptions and TODOs

| Marker | What must be done |
|---|---|
| **Never run on hardware** | The whole driver, including the init sequence, is still untested against a real sensor. The product-ID check passing is the first milestone. |
| `TODO(datasheet)` | Confirm the SPI delays against a PixArt datasheet. They now match Bitcraze's working values, but no datasheet exists in `documents/`. |
| ~~`TODO(pins)`~~ | **Done 2026-08-29.** SPI pins 18/8/9/10 confirmed against the airframe; SCLK is on an underside pad. |
| `TODO(bench)` | `PMW3901_MIN_SQUAL = 40` against real surfaces. The 2 Hz serial log in `nav_estimator.c` is how you read `squal` to do this. |

Related, in `nav_estimator`: `FLOW_COUNTS_PER_RAD` is also unmeasured, and the gyro-compensation
axis pairing and signs are unverified. **All of these must be right before velocity readings mean
anything — and now that init succeeds, the velocity loop will engage on whatever they currently
say.** See the warning at the top of this file.

## Dependencies

`esp_driver_spi`, `esp_driver_gpio`. Consumed by `nav_estimator`.

## Files

| File | Contents |
|---|---|
| `include/pmw3901_driver.h` | `pmw3901_motion_t`, three public functions, and the "not complete" warning. |
| `pmw3901_driver.c` | Pin/timing constants, manual CS, register read/write, init, the stub, motion read. |
| `CMakeLists.txt` | Component registration. |
