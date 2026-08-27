# `camera_sd` — photogrammetry capture and the viewfinder

Drives the XIAO ESP32-S3 Sense's onboard camera and writes JPEGs to the SD card. It exists to
produce **capture sets for photogrammetry / 3D Gaussian Splatting reconstruction**, with a
low-resolution viewfinder bolted on so the pilot can aim.

It is completely independent of flight. Nothing here can stop the drone flying, and nothing here
runs on the same core as the control path.

## Two ideas shape the whole component

### 1. Exposure, white balance and gain are **locked**

Automatic exposure is actively harmful for reconstruction. As the drone orbits an object,
auto-exposure re-meters on every frame and the same surface comes out a different brightness in
every shot. Structure-from-motion pipelines interpret that as a change in the *scene* rather than
a change in the *camera*, and the reconstruction degrades.

**Constant exposure across the whole capture set matters more than any individual frame being
correctly exposed.**

### 2. SD writes never touch the HTTP handler or the control path

A FAT write to a cheap SD card can block for **hundreds of milliseconds** when the card decides to
do wear levelling. Inside `/api/capture` that would stall the HTTP server → stall the dashboard's
20 Hz input POSTs → trip the browser watchdog mid-flight.

So `/api/capture` pushes a token onto a queue and returns. A dedicated task **pinned to core 1**
does the grab-and-write, well away from the UART and HTTP tasks on core 0.

## Exposure calibration — measured, then pinned

`camera_sd_calibrate_exposure()` runs once at boot:

1. Hand control back to the sensor: AEC, AGC, AEC2 on, gain ceiling 2×.
2. Apply `CAM_AE_LEVEL` (−1) as a bias while measuring. **Negative on purpose** — for
   reconstruction, slightly dark is recoverable but a blown highlight is gone for good, and a
   shorter exposure also means less motion blur from a moving drone.
3. Grab and discard `CAM_AEC_CALIBRATION_FRAMES` (8) frames with `CAM_AEC_SETTLE_MS` (60 ms)
   between them — auto-exposure converges over several frames, so throw some away before
   believing it.
4. Read back `sensor->status.aec_value` and `agc_gain`.
5. Turn the automatics **off**, then **write the measured values back explicitly**.
6. Remember them in `locked_aec_value` / `locked_agc_gain`.

**Why measure rather than hardcode.** "The same exposure every shot" is not the same thing as "a
number hardcoded months earlier" — that only works if the lighting happens to match whatever was
guessed. This gets both properties: correct for the actual scene, *and* identical across the set.

> **Point the drone at a representative part of the scene while it boots.** Everything after this
> call is fixed until the next reboot.

**Step 5's order matters and is repeated everywhere the sensor is reconfigured.** Disabling the
AEC loop can leave the registers at whatever the last iteration wrote, so the values are always
re-asserted after the automatics are switched off — never before.

Calibration is **non-fatal**: if every frame grab fails, it leaves auto-exposure running rather
than locking to a garbage value, logs an error, and carries on. Shots will vary between frames,
but the camera still works and the drone still flies.

## Two modes on one sensor

| | **Capture** | **Preview / viewfinder** |
|---|---|---|
| Resolution | QXGA (OV3660) / UXGA (OV2640), detected at runtime | QVGA 320×240 |
| JPEG quality | 10 | 14 |
| Exposure | **Locked** | Free-running AEC/AGC |
| Rate | On request | ~8 fps (120 ms) |

The **capture task alone owns the sensor.** The HTTP handler only ever sets flags
(`preview_requested`) — it never calls into the camera driver. That is why a slow frame grab can
never delay a control POST.

**Preview exposure is deliberately left free.** The preview is not part of the capture set, so its
exposure need not match anything, and leaving AEC on keeps the viewfinder usable as you fly
between differently lit parts of a room.

Preview is **off by default and should stay off unless you are framing a shot** — the radio is the
expensive part of this board, and preview is the only thing that makes it transmit continuously.
QVGA costs roughly 40× less data per frame than streaming capture resolution would, which is what
makes a viewfinder viable at all.

### Capturing while previewing

`camera_sd_capture_one()` handles the round trip:

```
was_previewing?
  ├─ enter_capture_mode()          switch resolution, re-impose the locked exposure
  ├─ discard CAM_CAPTURE_SETTLE_FRAMES (2)   frames in flight were exposed under the old timing
  ├─ write_one()
  └─ enter_preview_mode()          back to the viewfinder
```

The settle frames matter: without them, the first shot after each preview toggle comes out wrongly
exposed.

**Why the switch-back is in the wrapper, not in `write_one()`:** the write path has several early
returns for grab and card failures. Doing the restore here means no failure path can skip it —
otherwise the sensor would be left stuck at capture resolution and the viewfinder would silently
die after the first failed shot.

Net effect: **shots taken with preview on are identical to shots taken with it off.**

## The capture task

```c
wait = preview_active ? 120 ms : portMAX_DELAY;
xQueueReceive(capture_queue, &token, wait);
```

With preview off this blocks **indefinitely and costs no CPU at all** — the normal in-flight case.
With preview on it wakes every frame interval. Either way a capture request is serviced the moment
it arrives.

`CAPTURE_TOKEN_WAKE` carries no work. It exists purely to break the `portMAX_DELAY` wait so a
preview toggle is noticed immediately instead of at the next capture.

Priority **3** — below the UART tasks (7, 6), the HTTP server (5) and `gs_link` (4). Core 1, alone.

## Hardware

### Camera pins — fixed by the board

| Signal | GPIO | | Signal | GPIO |
|---|---:|---|---|---:|
| XCLK | 10 | | D7…D0 | 48, 11, 12, 14, 16, 18, 17, 15 |
| SIOD (SCCB data) | 40 | | VSYNC | 38 |
| SIOC (SCCB clock) | 39 | | HREF | 47 |
| PWDN / RESET | −1 (not connected) | | PCLK | 13 |

These are **not free choices** — they are the XIAO Sense's board-to-board connector. Don't
"correct" them to a different ESP32-S3 camera board's mapping.

### SD card

| Signal | GPIO |
|---|---:|
| CLK | 7 |
| CMD | 9 |
| D0 | 8 |

Mounted at `/sdcard`. **This is the pin collision** that pushed the telemetry UART to GPIO 43/44 —
see [`uart_link`](../uart_link/README.md).

Files are written as `/sdcard/IMG_NNNNN.JPG`. `camera_sd_scan_existing_files()` finds the highest
existing index at boot so a re-flash doesn't overwrite a previous session's set.

Camera frame buffers live in PSRAM (`CAM_FB_COUNT` = 2) — octal PSRAM is required by
`sdkconfig.defaults` for exactly this reason.

## Everything is non-fatal

`camera_ok` and `sd_ok` are tracked **independently**. `camera_sd_init()` returns
`ESP_ERR_NOT_FOUND` if either failed, but `app_main()` logs it and carries on. The drone must fly
with a broken camera, a missing card, or both.

`camera_sd_get_stats()` surfaces `requested` / `written` / `failed` on the dashboard so you can
tell **mid-flight** whether shots are actually landing on the card, rather than discovering an
empty card afterwards.

## Public API

| Function | Called from | Blocking? |
|---|---|---|
| `camera_sd_init()` | `app_main` | startup |
| `camera_sd_start_task(core_id)` | `app_main` | no — should be core 1 |
| `camera_sd_request_capture()` | HTTP handler | **no** — queue push only |
| `camera_sd_get_stats(&out)` | HTTP handler | no |
| `camera_sd_set_preview(bool)` | HTTP handler | **no** — sets a flag |
| `camera_sd_preview_enabled()` | HTTP handler | no |
| `camera_sd_copy_preview(dst, cap)` | HTTP handler | 5 ms mutex; never touches the sensor |
| `camera_sd_camera_ok()` / `_sd_ok()` | HTTP handler | no |

## Timing and concurrency

| State | Mechanism |
|---|---|
| `stats` | `portMUX` spinlock — written by the capture task, read by the HTTP task |
| `preview_buffer` / `preview_length` | Mutex, 5 ms timeout on both sides |
| `preview_requested` | Plain `volatile bool` — single aligned byte, one writer |
| Capture requests | FreeRTOS queue, depth 8, zero-timeout send |
| Sensor registers | **No lock at all** — owned exclusively by the capture task |

That last row is the important one: the sensor needs no lock because exactly one task ever touches
it. Every cross-task interaction is a flag or a queue.

`camera_sd_copy_preview()` copies from a buffer the capture task refreshes in the background, so
it returns whatever the latest complete frame is and never waits on a grab. A dropped viewfinder
frame is not logged — that would be a log line every 120 ms.

## Unverified assumptions and TODOs

| Marker | Value | What to check |
|---|---|---|
| `TODO(bench)` | `CAM_AE_LEVEL = -1` | Exposure bias during calibration, range −2…2. Tune to your lighting. |
| `TODO(bench)` | `CAM_WB_MODE = 3` | Pick the white-balance preset matching your lighting. |

## Dependencies

**Private** (`PRIV_REQUIRES`): `esp_driver_sdmmc`, `fatfs`, `sdmmc`. The `esp32-camera` component
arrives via `idf_component.yml`, not this list. `camera_sd.h` needs only stdint/stdbool/esp_err/
freertos, so none of the above is re-exported.

## Files

| File | Contents |
|---|---|
| `include/camera_sd.h` | `camera_sd_stats_t`, nine public functions, the two shaping ideas. |
| `camera_sd.c` | Pins, exposure constants, calibration, the two modes, the capture task, SD mount and write. |
| `CMakeLists.txt` | Component registration. |
