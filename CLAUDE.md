# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A custom tiny-whoop quadcopter project: two independent ESP32-S3 firmware projects that fly the
drone together, plus the physical frame design and reference docs. There is no top-level build —
each firmware project is built separately with its own `idf.py`.

- **`flight_controller/`** — runs on the drone itself (XIAO ESP32-S3), the 1 kHz sensor-fusion +
  cascaded-PID flight control loop. See `flight_controller/CLAUDE.md`.
- **`telemetry_module/`** — runs on a second XIAO ESP32-S3 Sense (camera + PSRAM + SD), hosts the
  pilot's Wi-Fi AP / web dashboard and relays control input over UART, plus independent
  photogrammetry camera capture. See `telemetry_module/CLAUDE.md`.
- **`shared_components/telemetry_uart/`** — the framed binary UART protocol shared by both
  firmware projects, pulled into each via `EXTRA_COMPONENT_DIRS` (not vendored — it's one copy on
  disk). **Any change here must be made with both firmware projects' CLAUDE.md context in mind**,
  since message types/payload structs are a contract between them.
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

When touching `shared_components/telemetry_uart`, changes ripple to both firmware projects —
check both CLAUDE.md files' "shared protocol" sections and grep the other project for the other
side of any change before making one.
