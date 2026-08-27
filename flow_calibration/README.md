# `flow_calibration` — PMW3901 optical flow scale bench tool

An ESP-IDF project that runs on a **spare ESP32-S3**, not on the drone. It measures
`FLOW_COUNTS_PER_RAD` for
[`../flight_controller/components/nav_estimator/nav_estimator.c`](../flight_controller/components/nav_estimator/nav_estimator.c),
which currently holds the guess `500.0f`.

Sibling of [`../tof_calibration`](../tof_calibration/) and built the same way. Bench tooling, not
aircraft firmware.

## ⚠ Read this before flashing the drone

Completing the PMW3901 driver removed a safety property the project was relying on.

While `pmw3901_init()` always failed, `velocity_valid` was permanently false and the velocity
outer loop could never engage. That is no longer true. With the ToF now supplying
`altitude_valid`, `flight_control` **will** engage velocity hold using the unmeasured
`FLOW_COUNTS_PER_RAD` and the unverified gyro-compensation signs. A sign error there is positive
feedback: the drone accelerates away from the hold point instead of settling on it.

**Do not reflash `flight_controller` until:**

1. `FLOW_COUNTS_PER_RAD` has been measured with this tool, and
2. the translation signs check out here, and
3. the rotation signs check out on the drone (pitch in place → `velocity_x` stays near zero).

## The math — and a correction to the documented procedure

`nav_estimator.c` used to say `angle_moved_rad = atan2(D, h)`. **That was wrong.** The sensor
accumulates frame-to-frame image shifts, so a slide of distance `D` at height `h` accumulates
`∫(v/h)dt = D/h`. `atan(D/h)` is the angle subtended at the *end* position, which is not what
accumulates.

It is also what the downstream arithmetic requires: `velocity = (counts/K)/dt · altitude` only
recovers the true `D/dt` when `counts/K = D/h`. The "rad" in the constant's name is really tangent
units (pixel displacement) — the pinhole relation.

```
FLOW_COUNTS_PER_RAD = accumulated_counts × h / D
```

With the old comment's own numbers (`D=200, h=300`), `atan` gives `0.588` against the correct
`0.667` — `K` came out 13% high and estimated velocity 13% low. The comment in `nav_estimator.c`
has been corrected.

Keeping `D/h ≲ 0.3` makes the two forms agree to ~1%, so the defaults here are specified that way
and the distinction can't bite regardless of lens non-idealities.

## What you need

| Item | Notes |
|---|---|
| Spare ESP32-S3 | Pins configurable — see below |
| PMW3901 breakout | |
| A rigid slide | A straightedge, a drawer runner, a book edge — anything that constrains motion to a straight line **without rotating** |
| A spacer of known height | Sets `h`. Measure from the **lens**, not the PCB |
| A textured surface | Newspaper, patterned rug, wood grain. Not plain carpet, not glossy laminate, not blank paper |
| A ruler | `K = counts·h/D`, so errors in `h` and `D` contribute equally and linearly |

### The whole field of view must see ONE flat plane

The most damaging rig mistake, because it produces a plausible number rather than an obvious
failure. The sensor correlates its entire 30×30 frame at once, and image shift scales as `ΔX/h` —
so two surfaces at different depths in the same frame flow at *different rates*, and you get one
blended answer weighted by whichever has more contrast.

It is disproportionately bad, because a nearer surface flows faster. A sensor overhanging a table
edge to look at the floor 890 mm below, catching the tabletop (155 mm below) in just 10% of the
frame:

```
0.9 × (1/890) + 0.1 × (1/155) = 0.001656   vs   true 1/890 = 0.001124   →  +47%
```

Ten percent intrusion, ~47% error. `squal` may or may not flag it.

Clearance needed is `h × tan(21°)` — at 890 mm that is 341 mm of horizontal clearance to the
nearest other surface, **plus** the slide travel. Which is why looking down at a nearby surface
(newspaper on the table, sensor on a short stand above it) is almost always the better rig than
looking down at something far away past an obstruction.

Field-of-view patch diameter is `2 × h × tan(21°)`: 119 mm at h=155, 307 mm at h=400, 683 mm at
h=890. The textured area must cover the whole patch.

## Wiring

Defaults here are set for a **XIAO ESP32-S3**, per
[`../documents/XIAO_ESP32-S3_front_pinout.png`](../documents/XIAO_ESP32-S3_front_pinout.png):

| Signal | GPIO | Pad | Note |
|---|---|---|---|
| SCK | 7 | D8 | silkscreen `SPI0_SCK` |
| MISO | 8 | D9 | `SPI0_MISO` |
| MOSI | 9 | D10 | `SPI0_MOSI` |
| CS | 4 | D3 | any free pin — bit-banged by the driver, not the SPI peripheral |
| VIN | — | 3.3V-OUT | **not** VBUS; the PMW3901 is a 3.3 V part |
| GND | — | GND | |

Note these are **not** the driver's own defaults (`SCLK 18 / CS 10`). The XIAO ESP32-S3 breaks out
only 11 GPIOs — 1–9, 43, 44 — so GPIO 18 and GPIO 10 do not physically exist on it. See the
"drone pin map" warning below.

The "SPI0" labels are Seeed's Arduino naming. On the ESP32-S3, SPI0/SPI1 belong to flash and
PSRAM; the driver uses `SPI2_HOST` and the GPIO matrix routes it to those pads.

**Change them in [`CMakeLists.txt`](CMakeLists.txt)**, not in the driver:

```cmake
idf_build_set_property(COMPILE_DEFINITIONS "PMW3901_PIN_SCLK=18" APPEND)
```

Those five `#define`s are `#ifndef`-guarded in `pmw3901_driver.c`, so this build overrides them
and the drone's firmware is untouched. `main.c` prints the compiled-in values at boot, so the
serial log always shows what actually got built.

This differs from the ToF tool, where `main.c` chose the I²C pins directly. `vl53l1x_driver`
*borrows* a bus; `pmw3901_driver` *creates* one, so its pins are compile-time only.

Avoid the strapping pins (0, 3, 45, 46) and the USB-JTAG pins (19, 20).

## Frame view — check what's actually in shot first

Before trusting any trial, verify the field of view is clean. In [`main/main.c`](main/main.c):

```c
#define CAL_FRAME_VIEW 1
```

Rebuild, reflash, and the tool prints the sensor's raw 35×35 image as ASCII art once a second
instead of running trials. Contrast is auto-scaled to the frame's own min/max, because the
PMW3901's raw levels sit in a narrow band that shifts with lighting — a fixed ramp just renders
flat grey.

- **What you want:** uniform texture filling the whole frame.
- **What to eliminate:** a straight edge, a bright blob, or a dark region that *doesn't move*
  when you slide the rig. That's the table edge or the clamp.

This is a **separate build on purpose.** Entering frame mode permanently disables motion
reporting until the sensor is power-cycled, so the modes can't be confused at runtime. Set
`CAL_FRAME_VIEW` back to `0`, rebuild and reflash before running trials.

A frame takes ~0.5 s to read (1225 pixels, two SPI reads each), so it's a static check, not a
live viewfinder.

**The view is wider than the region that actually matters.** The readout is 35×35, but the
correlator computes flow over 30×30 — so clutter right at the frame edge may not be corrupting
anything. The check errs on the safe side: a clean frame definitely means a clean measurement, but
a slightly cluttered edge isn't automatically a reason to rebuild the rig. Clutter anywhere near
the centre is.

## Running it

```bash
idf.py set-target esp32s3
```

```bash
idf.py -p /dev/ttyACM0 flash monitor
```

(Source your ESP-IDF `export.sh` first.)

Set `CAL_HEIGHT_MM` and `CAL_SLIDE_MM` in [`main/main.c`](main/main.c) to match your rig — they go
directly into the result.

There is no countdown and no fixed window. **Slide mark to mark in about a second, let go, and
wait for the result to print.** The trial ends itself once the rig has moved and then been
stationary for 1.2 s. Then slide it back — that's simply the next trial, in the opposite
direction, and equally valid. Take the magnitude.

Two things the tool now checks for you, both learned the hard way:

- **Reversal.** A clean one-way slide ends at its own peak excursion. If the net comes back well
  below the peak, the rig moved again before settling — usually the return trip starting early —
  and the net is the *difference* of two slides, not one. The report says so; discard that trial.
- **Counts per read.** The sensor reports *integer* deltas. A slide slow enough that each read
  sees a fraction of a count can lose displacement and bias `K` **low**. A 30 mm slide taken over
  10 seconds is only ~0.09 counts per read — almost every read returns zero. Aim comfortably above
  1; the report warns below 0.5. This is why "slide briskly" matters, and it is *not* fixable by
  polling slower — the delta register accumulates between reads, so the loss is internal to the
  sensor and depends on speed alone.

**Keep the sensor flat.** There is no IMU on this rig, so there is no gyro compensation. Any tilt
or rotation during the slide goes straight into the measured constant as error.

## Reading the output

Each trial reports accumulated `dx`/`dy`, a `K` candidate for each axis, and surface-quality
stats. Slide along one axis at a time: a forward slide makes `dx` the meaningful number, a
rightward slide makes it `dy`.

The tool **accumulates low-quality reads rather than discarding them** — dropping them would throw
away real displacement and bias `K` low. Instead it reports how many were below the advisory
`squal` threshold, so a bad surface shows up as a reason to re-run rather than as a silently wrong
number.

It also gives you the translation half of the sign check for free: `nav_estimator` expects forward
→ `+dx` and right → `+dy`. The trial output states what was actually observed.

Sanity check: the published figure for this part's ~42° field of view over 30×30 pixels lands near
500. That is a smell test, not a target — measure your own unit.

## Files

| File | Contents |
|---|---|
| `CMakeLists.txt` | Pulls in `pmw3901_driver`; **SPI pin overrides live here** |
| `sdkconfig.defaults` | Minimal, plus the 1 kHz tick the driver's bring-up delays need |
| `main/main.c` | Trial loop and the arithmetic |
