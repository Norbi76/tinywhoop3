# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

The telemetry/companion-computer firmware for a custom tiny-whoop quadcopter, running on a XIAO
ESP32-S3 Sense (camera + PSRAM + SD card) under ESP-IDF. It is the pilot-facing half of a two-board
system: it hosts a Wi-Fi SoftAP and web dashboard, turns held-button input into flight setpoints,
relays them over UART to a separate board — `../flight_controller` — which runs the actual 1 kHz
control loop, and independently drives an onboard camera for photogrammetry capture sets. The wire
protocol shared with `flight_controller` lives in `../shared_components/telemetry_uart` (pulled in
via `EXTRA_COMPONENT_DIRS` in the top-level `CMakeLists.txt`, not vendored) — changes to message
types or payload structs must stay compatible with, or be made alongside changes to,
`flight_controller`.

Sibling directories under `../` (`documents/`, `tinywhoop_frame/`) are reference material, not part
of this build.

## Build / flash / monitor

Standard ESP-IDF project (`idf.py`), targeting `esp32s3`. There is no test suite.

```bash
idf.py build
idf.py -p /dev/ttyACM0 flash
idf.py -p /dev/ttyACM0 monitor
idf.py -p /dev/ttyACM0 flash monitor
idf.py menuconfig
```

`sdkconfig.defaults` is authoritative (8 MB flash / 4 MB app partition, octal PSRAM required for the
camera frame buffers, 1000 Hz FreeRTOS tick so the 50 Hz UART cadence is exact). `sdkconfig`/
`sdkconfig.old` are generated and gitignored; delete and rebuild to regenerate from defaults if they
drift. `partitions.csv` has no storage partition — photos go to the SD card, not flash.

### Dashboard development without hardware

`tools/mock_server.py` serves `components/http_server/www/index.html` directly off disk and
reimplements every `/api/*` endpoint plus a crude rigid-body flight sim, so the dashboard can be
iterated on without flashing a board or having a flight controller attached:

```bash
python3 tools/mock_server.py          # http://localhost:8080, stdlib only
```

It mirrors the ramp/decay setpoint integration, the arming gate, and the browser watchdogs from
`control_state.c` byte-for-byte, and exposes fault-injection endpoints (`/mock/link`,
`/mock/camera`, `/mock/sd`, `/mock/reset`) for exercising the dashboard's unhappy paths. If you
change the setpoint math or a `/api/*` response shape in the firmware, update this file's mirror of
it too, or dashboard behavior will diverge between the mock and the real board.

## Architecture

### Startup and core assignment (`main/main.c`)

`app_main()` brings components up in a fixed, dependency-driven order: `control_state` first (so
every later module has somewhere to read/write), then the UART link to the flight controller
(before Wi-Fi — this is what actually flies the drone), then camera/SD (**non-fatal**: a missing
card or failed camera probe is logged and ignored, never blocks flight), then Wi-Fi AP, then the
HTTP server last.

- **Core 0**: Wi-Fi/lwIP, the HTTP server, and both UART tasks — the whole control path on one core
  so the 50 Hz UART TX cadence and the 20 Hz dashboard input POSTs are scheduled against each other
  by one scheduler with no cross-core latency.
- **Core 1**: the camera capture task alone. Grabbing a JPEG out of PSRAM and writing it to the SD
  card via FATFS is a long, bursty, blocking job; isolating it means a slow card can never delay a
  control frame.

### Control state (`components/control_state`)

Turns the dashboard's held-button booleans into flight setpoints: while a button is held, an axis
**ramps** toward its limit; released, it **decays** back to neutral (decay rate ~2x ramp rate, so
letting go always settles the drone faster than pressing moved it). Throttle is the exception — a
persistent trim with no decay branch, since a decaying throttle is unflyable with a button
interface. `control_state_update()` runs at a fixed 50 Hz off the UART TX task, not from the HTTP
handler, so setpoints advance at a known rate regardless of how irregularly the browser posts.

Two watchdogs live here: `INPUT_TIMEOUT_US` (500 ms — no POST means release all directional inputs
and level out, but keep the throttle trim so altitude holds) and `DISARM_TIMEOUT_US` (3 s — a
longer backstop that drops the arm request outright, since the UART link would otherwise keep
sending valid 50 Hz frames forever and the flight controller's own link watchdog would never fire).

### UART link (`components/uart_link`, protocol in `../shared_components/telemetry_uart`)

Two FreeRTOS tasks, both pinned to core 0: TX at a **fixed 50 Hz** regardless of pilot activity
(higher priority than RX — a missed control frame risks the flight controller's 300 ms link
watchdog disarming it), piggybacking up to `GAINS_PER_CYCLE` queued PID gain updates per cycle; RX
continuous, using `uart_telemetry_read_message_resync()` so one corrupted byte costs a single frame
rather than flushing the whole buffer (important at 50 Hz — a full flush is enough sustained loss to
trip the flight controller's watchdog).

Framed binary protocol: `[start_byte | msg_type | payload_len | seq] [payload 0..64B] [crc16 |
end_byte]`. Message types and payload structs (`telemetry_control_payload_t`,
`telemetry_status_payload_t`, `telemetry_gains_payload_t`) are the shared contract with
`flight_controller`; `_Static_assert`s in `telemetry_uart.h` enforce every payload fits in one
frame. **UART pins are TODO/unconfirmed** (`uart_link.h`: GPIO 43/44) — the XIAO Sense's SD card
uses GPIO 7/8/9, which collided with an earlier pin choice; check against actual wiring before
flashing a new harness.

### Camera and SD (`components/camera_sd`)

Runs on its own core-1 task, gated entirely by non-fatal init: `camera_ok`/`sd_ok` are tracked
independently and the drone must fly even if either is missing. Pins are fixed by the XIAO Sense's
board-to-board connector (not free choices — don't "correct" them to a different ESP32-S3 camera
board's mapping). Two operating modes on the same sensor, switched by the capture task (which alone
owns the sensor — the HTTP handler only ever sets flags):

- **Capture** (QXGA, quality 10): exposure and white balance are **measured then locked** at boot
  (`camera_sd_calibrate_exposure()` runs the sensor's own AEC for a few frames against the real
  scene, then pins the result) so every photo in a set shares identical exposure/color — required
  for photogrammetry/Gaussian Splatting reconstruction, where brightness or color drift gets
  misread as geometry.
- **Preview/viewfinder** (QVGA, ~8 fps, free-running auto-exposure): exists only so the pilot can
  aim the camera; quality is irrelevant and it never touches the locked capture exposure. Served
  one JPEG per `GET /preview.jpg` request rather than as an MJPEG stream, because ESP-IDF's HTTP
  server is single-task-serial and a long-lived stream would stall the 20 Hz control POSTs.

Captures are requested via a zero-timeout queue send from the HTTP handler and actually grabbed/
written on the capture task, so an SD write (hundreds of ms) never blocks the HTTP server.

### Wi-Fi AP (`components/wifi_ap`)

SoftAP, not infra mode. Power save is explicitly disabled after `esp_wifi_start()` — the default
`WIFI_PS_MIN_MODEM` adds tens of ms of beacon-interval jitter to every control POST, which is
unflyable by hand. TX power is deliberately turned down from the default (a room-range drone doesn't
need wall-penetrating range, and radio TX is the largest current draw on a small battery) —
independent of the power-save setting, so it costs range/margin, not latency. Credentials
(`WIFI_AP_SSID`/`WIFI_AP_PASSWORD` in `wifi_ap.h`) are a known-weak placeholder, flagged TODO before
flying anywhere with other people around.

### HTTP server (`components/http_server`)

`GET /` serves `www/index.html`, which is **compiled into the binary** via `EMBED_FILES` in
`CMakeLists.txt` (not read from the SD card), so the dashboard works with no card fitted and can't
go stale relative to the firmware. All `POST` bodies are small flat JSON objects parsed with
deliberately hand-rolled scraping (`json_flag`/`json_number` in `http_server.c`), not a JSON
library — the payloads never nest, and pulling in cJSON would cost heap/parse time on the 20 Hz hot
path for no benefit. If the dashboard ever needs nested JSON, replace these rather than extending
them. Endpoints: `/api/input` (20 Hz button state), `/api/arm`, `/api/mode`, `/api/capture`,
`/api/gains`, `/api/status` (GET, everything the dashboard displays), `/api/preview`,
`/preview.jpg`.

### Shared telemetry protocol vs. `flight_controller`

`../shared_components/telemetry_uart` is the wire contract between this board and
`../flight_controller`. `telemetry_loop_id_t` ordering must match `pid_loop_id_t` on the flight
controller side — it addresses one of 8 PID instances for live gain tuning from the dashboard.
Flight mode enum (`telemetry_flight_mode_t`) and status/control flag bits are likewise shared and
must be kept in sync across both projects; grep `flight_controller` for the other side of any
protocol change before making one here.
