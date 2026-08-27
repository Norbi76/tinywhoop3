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

It also bridges the flight controller's PID tuning traffic onto Wi-Fi over UDP, so the desktop tool
in `../tools/ground_station` can tune loops in flight. That path is a pure relay — this board never
interprets those messages.

Sibling directories under `../` (`documents/`, `tinywhoop_frame/`) are reference material, not part
of this build. `../tools/ground_station` is project code but not part of this build.

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
card or failed camera probe is logged and ignored, never blocks flight), then Wi-Fi AP, then
`gs_link` (**non-fatal**, and after the AP so lwIP has a netif to bind to), then the HTTP server
last — it is what starts accepting pilot input, so everything it touches must already exist.

The fatal/non-fatal split is consistent: anything the pilot needs in order to fly (`control_state`,
`uart_link`, `wifi_ap`, `http_server`) returns from `app_main` on failure; anything that only makes
the drone more useful (`camera_sd`, `gs_link`) logs and continues.

Task priorities on core 0, highest first — the ordering is a single statement that the closer
something is to keeping the drone armed and controllable, the higher it sits:
UART TX (7) > UART RX (6) > HTTP server (5) > `gs_link` (4). The camera task is 3, on core 1.

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
watchdog disarming it), piggybacking up to `GAINS_PER_CYCLE` (2) queued PID gain updates and
`UPLINK_PER_CYCLE` (4) queued ground-station frames per cycle; RX continuous on 5 ms wakeups,
draining up to `RX_DRAIN_PER_CYCLE` (64) frames before flushing to the ground station and using
`uart_telemetry_read_message_resync()` so one corrupted byte costs a single frame rather than
flushing the whole buffer (important at 50 Hz — a full flush is enough sustained loss to trip the
flight controller's watchdog).

All three of those bounds exist for the same reason: optional traffic must never push the control
frame late, and a flood of PID debug frames must never monopolise the RX task. Keep them bounded.

**The TX task is the only task in this firmware that calls `uart_telemetry_send_message()`**, and
that is a hard rule, not a convention: the function bumps a shared sequence counter and then issues
three separate `uart_write_bytes()` calls, so a second caller interleaves its bytes into another
task's frame and produces CRC failures on the link that flies the drone. Everyone else queues.

Framed binary protocol: `[start_byte | msg_type | payload_len | seq] [payload 0..64B] [crc16 |
end_byte]`. Message types and payload structs (`telemetry_control_payload_t`,
`telemetry_status_payload_t`, `telemetry_gains_payload_t`) are the shared contract with
`flight_controller`; `_Static_assert`s in `telemetry_uart.h` enforce every payload fits in one
frame. **UART pins are TODO/unconfirmed** (`uart_link.h`: GPIO 43/44) — the XIAO Sense's SD card
uses GPIO 7/8/9, which collided with an earlier pin choice; check against actual wiring before
flashing a new harness.

### Ground-station bridge (`components/gs_link`)

A UDP bridge (port 14550) tunnelling the flight controller's PID tuning traffic to the Python tool
in `../tools/ground_station`. **This module does not interpret any of it** — frames off the UART are
batched into datagrams and datagrams coming back are split into records and queued for the UART TX
task. It is a pipe, and it runs completely independently of the dashboard: either works without the
other.

Three things here are non-obvious and load-bearing:

- **The UDP wire format is not the UART frame.** One datagram carries N records of
  `msg_type(1) | payload_len(1) | seq(2) | payload`. No start byte, no CRC, no end byte — UDP
  already provides framing and error detection, and re-wrapping would spend ~20% of the link on
  redundant bytes.
- **Batching, not per-frame sending.** `gs_link_forward()` only appends; `gs_link_flush()` sends,
  once per RX drain. At 500 Hz, one datagram per frame would be 500 packets/second and the SoftAP
  chokes on packet *rate* long before bitrate.
- **The uplink queue lives in `gs_link`, not `uart_link`** — deliberately, so the dependency runs
  one way (`uart_link` privately requires `gs_link`; `gs_link` knows nothing about `uart_link`).
  Putting it in `uart_link` would create a cycle. The queue itself exists because of the
  single-writer rule above.

Peer discovery is by latching: any inbound packet becomes the current peer before its contents are
even examined, which is what lets the ground station restart or change IP without touching the
drone. With no peer, outbound datagrams are dropped silently — that is the normal state.

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
`../flight_controller`. Flight mode enum (`telemetry_flight_mode_t`) and status/control flag bits
are shared and must be kept in sync across both projects; grep `flight_controller` for the other
side of any protocol change before making one here.

There are **two independent loop-id enums** addressing the same 8 PID instances, and they are
deliberately not unified: `telemetry_loop_id_t` (inner-to-outer) is the web dashboard's, with its
values baked into `index.html`; `pid_loop_id_t` (outer-to-inner) is the Python ground station's.
`pid_registry_bind()` on the flight controller reconciles them. Renumbering either silently retunes
the wrong axis from the browser.

Message types 20–23 (`PID_DEBUG` / `PID_GAINS` / `PID_SELECT` / `PID_INJECT`) pass through this
board **uninterpreted** — see `components/gs_link` above. Their payloads carry exact-size
`_Static_assert`s because `tools/ground_station/protocol.py` unpacks them with hardcoded `struct`
formats.

## Per-component documentation

Every component has a `README.md` (`components/<name>/README.md`, plus `main/README.md`) covering
its public API, timing/concurrency model, hardware dependencies, and its own TODO table. Read the
component's README before editing it; this file is the map, those are the detail. Every source file
also carries a header comment stating its purpose and mechanism.

`managed_components/` (esp32-camera, esp_jpeg) is third-party — do not edit it.
