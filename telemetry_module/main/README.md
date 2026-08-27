# `main` — startup ordering and core assignment

`app_main()` and nothing else. Every subsystem lives in a component; this file's only job is to
bring them up in the right order and put them on the right core.

Both of those are load-bearing, which is why this is a README rather than a footnote.

## Core assignment

| Core | What runs there |
|---:|---|
| **0** | Wi-Fi + lwIP (the IDF pins them here anyway), the HTTP server, both UART tasks, `gs_link` |
| **1** | The camera capture task, **alone** |

**Why the whole control path is on one core.** The 50 Hz UART transmit cadence and the 20 Hz
dashboard input POSTs are two halves of the same loop. Putting them on one core means one
scheduler arbitrates between them, with priorities that actually mean something and no cross-core
latency in between.

**Why the camera is isolated.** Grabbing a UXGA JPEG out of PSRAM and pushing it through FATFS
onto an SD card is a long, bursty, blocking job. On the other core, a slow card cannot delay a
control frame no matter how long it takes.

Priorities on core 0, highest first:

```
  7  UART TX       missing a slot risks the flight controller's 300 ms watchdog
  6  UART RX       a late status frame only makes the dashboard stale
  5  HTTP server   serving the dashboard must never delay a control frame
  4  gs_link       a stalled ground station must never delay anything
  ----------------------------------------------------------------------
  3  camera task   (core 1)
```

That ordering is a single consistent statement: **the closer something is to keeping the drone
armed and controllable, the higher it sits.**

## Startup order

```
 1. control_state_init()                      FATAL
 2. uart_link_init() + uart_link_start(0)     FATAL
 3. camera_sd_init() + start_task(1)          non-fatal
 4. wifi_ap_init()                            FATAL
 5. gs_link_start()                           non-fatal
 6. http_server_start()                       FATAL
```

Each position is chosen, not incidental:

1. **`control_state` first** so every later module has somewhere valid to read from and write to.
   It depends on nothing.
2. **UART link before Wi-Fi.** This is the link that actually flies the drone. If it cannot come
   up there is no point serving a dashboard.
3. **Camera and SD third, non-fatal.** Photography is the mission, but flight is the prerequisite.
   A missing card or a failed camera probe is logged, surfaced on the dashboard, and otherwise
   ignored. The capture task is started **even if init partly failed** —
   `camera_sd_request_capture()` rejects requests on its own when the hardware is missing, and
   having the task running keeps the shutdown paths uniform.
4. **Wi-Fi AP fourth**, because everything after it needs a network.
5. **`gs_link` after the AP** so lwIP has a netif to bind its UDP socket to. Non-fatal for the same
   reason as the camera: it is a tuning tool, not a flight requirement.
6. **HTTP server last**, because it is the thing that starts accepting pilot input, and everything
   it touches has to exist before the first request arrives.

## Fatal vs non-fatal

The split is consistent and worth stating explicitly:

| Fatal (returns from `app_main`) | Non-fatal (logs and continues) |
|---|---|
| `control_state`, `uart_link`, `wifi_ap`, `http_server` | `camera_sd`, `gs_link` |

**Anything the pilot needs in order to fly is fatal. Anything that only makes the drone more
useful is not.** The same principle appears on the flight controller side, where the ToF and
optical flow sensors are non-fatal and their loops simply disengage.

## What the log tells you at the end

```
Startup complete.
  Connect to Wi-Fi 'TinyWhoopAP' and open http://192.168.4.1/
  Camera: OK   SD card: OK
```

That last line is the quickest way to tell whether a capture session is going to work before you
take off — it is also mirrored into `/api/status` as `camera_ok` / `sd_ok`.

## Files

| File | Contents |
|---|---|
| `main.c` | `app_main()`, the core-assignment comment, the six init steps. |
| `CMakeLists.txt` | Depends on all six components. |

## See also

Component READMEs: [`control_state`](../components/control_state/README.md) ·
[`uart_link`](../components/uart_link/README.md) ·
[`gs_link`](../components/gs_link/README.md) ·
[`wifi_ap`](../components/wifi_ap/README.md) ·
[`http_server`](../components/http_server/README.md) ·
[`camera_sd`](../components/camera_sd/README.md) ·
[`telemetry_uart`](../../shared_components/telemetry_uart/README.md)

Development without hardware: [`../tools/mock_server.py`](../tools/mock_server.py).
