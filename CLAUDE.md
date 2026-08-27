# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A custom tiny-whoop quadcopter project: two independent ESP32-S3 firmware projects that fly the
drone together, plus a bench tool that also builds as firmware, the physical frame design and
reference docs. There is no top-level build — each firmware project is built separately with its
own `idf.py`.

- **`flight_controller/`** — runs on the drone itself (**ESP32-S3 Super Mini**), the 1 kHz sensor-fusion +
  cascaded-PID flight control loop. See `flight_controller/CLAUDE.md`.
- **`telemetry_module/`** — runs on a second XIAO ESP32-S3 Sense (camera + PSRAM + SD), hosts the
  pilot's Wi-Fi AP / web dashboard and relays control input over UART, plus independent
  photogrammetry camera capture. See `telemetry_module/CLAUDE.md`.
- **`shared_components/telemetry_uart/`** — the framed binary UART protocol shared by both
  firmware projects, pulled into each via `EXTRA_COMPONENT_DIRS` (not vendored — it's one copy on
  disk). **Any change here must be made with both firmware projects' CLAUDE.md context in mind**,
  since message types/payload structs are a contract between them.
- **`tools/ground_station/`** — a desktop PyQt6 tool for tuning the eight PID loops in flight: it
  plots one loop's setpoint/measurement/error/PID-terms live and reads and writes that loop's gains
  over UDP. Not part of either firmware build (`pip install -r requirements.txt`, `python3 gs.py`).
  Its traffic rides the same UART as everything else — `telemetry_module` forwards the `PID_*`
  message block verbatim between the UART and UDP without interpreting it. `protocol.py` hand-mirrors
  the packed structs in `telemetry_uart.h`; the exact-size `_Static_assert`s there are the only thing
  that catches a drift, so change both together. See `tools/ground_station/README.md`.
- **`tof_calibration/`** — a third ESP-IDF project, but a **bench tool, not aircraft firmware**. Runs
  on a spare ESP32-S3 to measure the VL53L1X offset that belongs in `VL53L1X_OFFSET_MM`
  (`flight_controller/components/vl53l1x_driver/vl53l1x_driver.c`, currently `0`/`TODO(bench)`). It
  reuses the real `vl53l1x_driver` component in place via `EXTRA_COMPONENT_DIRS` — pointed at the
  component directory itself, not at `flight_controller/components/`, which would drag in
  components requiring `telemetry_uart` — so calibration runs under the exact ranging config that
  flies. See `tof_calibration/README.md`.
- **`flow_calibration/`** — the same idea for the optical flow sensor: measures
  `FLOW_COUNTS_PER_RAD` (`flight_controller/components/nav_estimator/nav_estimator.c`, currently
  the guess `500.0f`) on a spare ESP32-S3. Reuses `pmw3901_driver` in place. Because that driver
  *creates* its SPI bus rather than borrowing one, the rig's pins are set via compile-time
  overrides in its `CMakeLists.txt`, not in `main.c`. **Read its README before reflashing the
  drone** — completing the PMW3901 init sequence removed the failure path that used to keep the
  velocity loop permanently disengaged. See `flow_calibration/README.md`.
- **`tinywhoop_frame/`** — KiCad PCB design for the physical airframe (`.kicad_pcb/.kicad_sch/.kicad_pro`,
  Gerber/drill fab files in `fab_files/`, dated zip snapshots in `tinywhoop_frame-backups/`). Not
  firmware; only relevant to pin/connector questions.
- **`documents/`** — reference material: IMU datasheet/register map, control architecture block
  diagram, board pinout image, oscilloscope captures. Not part of any build.

## Working in this repo

Read the relevant sub-project's `CLAUDE.md` (`flight_controller/CLAUDE.md` or
`telemetry_module/CLAUDE.md`) before making changes there — they document task layout, control
cascade, sensor fusion, and per-component architecture in depth. Don't duplicate that detail here;
this file is only the map between the pieces.

**Every component has its own `README.md`** (`<project>/components/<name>/README.md`, plus
`<project>/main/README.md` and `shared_components/telemetry_uart/README.md`). Those go a level
deeper than the CLAUDE.md files: exact constants and where they came from, the bench procedure for
each unverified sign convention, the locking strategy per piece of shared state, and why each
non-obvious choice was made. Read the component README before editing a component. Every source
file also carries a header comment stating its purpose and mechanism.

Third-party code that is **not** ours and should not be edited: `telemetry_module/managed_components/`
(esp32-camera, esp_jpeg), `flight_controller/components/vl53l1x_driver/st_uld/` (ST's Ultra Lite
Driver, own licence, compiled with `-Wno-error`), and
`flight_controller/components/pmw3901_driver/pmw3901_init_registers.h` (PixArt's power-up register
values, transcribed from Bitcraze's MIT-licensed driver — undocumented magic numbers; the only
correct way to change that table is to re-transcribe it from upstream).

When touching `shared_components/telemetry_uart`, changes ripple to both firmware projects —
check both CLAUDE.md files' "shared protocol" sections and grep the other project for the other
side of any change before making one.
