# `http_server` — the dashboard and the `/api/*` control surface

Serves the pilot's web dashboard and the nine endpoints behind it. Everything the pilot does —
arming, flying, changing mode, taking a photo, tuning gains — arrives here as an HTTP request.

The component itself holds **no state**. Every handler translates a request into a call on
`control_state`, `camera_sd` or `uart_link` and returns immediately.

## The dashboard is compiled into the binary

```cmake
EMBED_FILES "www/index.html"
```

`GET /` serves `index_html_start .. index_html_end`, symbols the linker creates from that embed —
**not** a file read from the SD card. Two consequences, both deliberate:

- The dashboard works with **no SD card fitted**.
- It **cannot go stale** relative to the firmware. There is no way to have a v2 dashboard talking
  to v1 firmware.

The cost is that changing the page needs a reflash. `tools/mock_server.py` exists to make that
cost irrelevant during development — it serves the same file straight off disk.

`Cache-Control: no-store` on `/`, `/api/status` and `/preview.jpg`. During development the page
changes on every flash, and **a stale cached dashboard controlling a drone is a genuinely bad
failure mode.**

The embed is declared in **this component's** `CMakeLists.txt`, not `main`'s — the dashboard
belongs to the component that serves it.

## Endpoints

| Method | Path | Does |
|---|---|---|
| GET | `/` | The dashboard |
| POST | `/api/input` | Eight held-button booleans. **20 Hz — the hot path.** |
| POST | `/api/arm` | `armed` / `kill` |
| POST | `/api/mode` | `mode` (0–2) and the `hold` toggle |
| POST | `/api/capture` | Queue one photo |
| POST | `/api/gains` | `loop`, `kp`, `ki`, `kd` for one PID |
| GET | `/api/status` | Everything the dashboard displays, as one flat JSON object |
| POST | `/api/preview` | `enable` — toggle the viewfinder |
| GET | `/preview.jpg` | One viewfinder frame as `image/jpeg` |

### Every write handler is a queue push, never the work itself

This is the pattern that makes the control path stay responsive:

| Handler | What it actually does |
|---|---|
| `/api/input` | Stores button state. **The ramp/decay integration runs on the 50 Hz UART TX task** — so setpoints advance at a fixed rate regardless of how irregularly the browser posts. |
| `/api/capture` | Pushes a token onto a queue. Does **not** grab the frame and does **not** write the file — an SD write blocks for hundreds of milliseconds and would stall the HTTP server, and therefore the 20 Hz control POSTs. |
| `/api/gains` | Queues; the frame goes out on the next 50 Hz TX cycle so it cannot delay a control frame. |
| `/api/preview` | Sets a flag. The capture task, which alone owns the sensor, applies the mode change. |

### `/api/arm` clears a pending kill

Any explicit arm *or* disarm also calls `control_state_set_kill(false)`. That is what lets the
pilot recover from a kill without power-cycling. The flight controller's own latch still requires
kill released **and** arm low before it will leave the killed state — so this cannot by itself
restart the motors.

### `/api/status` decides what "link up" means

```c
const bool link = have_status && (status_age_ms >= 0) && (status_age_ms < 500);
```

A status frame from ten seconds ago must not read as a healthy link. `armed` is likewise reported
as `link && status.armed`, so a stale frame can never show the drone as armed.

## Why JSON parsing is hand-rolled

`json_flag()` and `json_number()` are `strstr` for `"key"`, then look at what follows the colon.
That is all.

Deliberately **not** a real parser. The only producer is our own dashboard, the payloads are three
or four flat keys, and pulling in cJSON to parse `{"roll_left":true}` would cost heap and parse
time on the 20 Hz hot path for no benefit.

Known limits, accepted because of the single-producer assumption: it will match a key inside a
string value, it does not validate structure, and `json_flag` treats only literal `true`
(optionally one leading space) as set.

**If the dashboard ever needs nested JSON, replace these with cJSON rather than extending them.**
The scraping approach does not generalise, and half-generalising it is worse than either end.

## `/preview.jpg` is one frame per request, not a stream

ESP-IDF's HTTP server services requests **serially on a single task**. A long-lived MJPEG
multipart response would sit in that task and delay the dashboard's 20 Hz control POSTs for as
long as the stream was open.

Discrete requests keep the control path responsive and degrade gracefully: if the link slows down
the viewfinder just gets choppier while control carries on unaffected.

The frame buffer is a **`static`** `CAM_PREVIEW_MAX_JPEG` (32 KB) array, not a stack allocation —
the HTTP task stack is nowhere near big enough, and only one request is served at a time anyway.

## Server configuration

| Setting | Value | Why |
|---|---|---|
| `core_id` | 0 | Alongside the UART tasks; away from the camera work on core 1 |
| `task_priority` | **5** | **Below both UART tasks (7 and 6)** — serving the dashboard must never delay a control frame |
| `stack_size` | 8192 | The status handler builds a 640-byte JSON buffer on the stack |
| `lru_purge_enable` | true | The dashboard opens several short-lived connections; purging the least recently used avoids running out of sockets during a long session |
| `max_uri_handlers` | 12 | 9 registered, room to grow |

`http_server_start()` is idempotent.

## Request body handling

`read_body()` caps bodies at `MAX_BODY_LEN` (512) and rejects anything larger with a 400. It
retries on `HTTPD_SOCK_ERR_TIMEOUT` and NUL-terminates, so the scraping helpers can use `strstr`
safely.

## Timing and concurrency

- All handlers run on the single ESP-IDF HTTP task (core 0, priority 5). They are therefore
  **serialised with respect to each other** — no handler can race another, which is why the
  `static` preview buffer is safe.
- Cross-task safety comes from the components being called, not from here: `control_state` has its
  own mutexes, `uart_link` and `camera_sd` have queues. No handler blocks on any of them.
- Startup order matters: `http_server_start()` is the **last** thing `app_main()` does, after the
  UART link, camera/SD and Wi-Fi are all up — every handler assumes its dependency already exists.

## Dependencies

**Private** (`PRIV_REQUIRES`): `esp_http_server`, `control_state`, `camera_sd`, `uart_link`,
`wifi_ap`. `http_server.h` exposes only `esp_err_t`, so none of that is re-exported to callers.

**Plus** `EMBED_FILES "www/index.html"`.

## Files

| File | Contents |
|---|---|
| `include/http_server.h` | Two functions: `http_server_start()`, `http_server_stop()`. |
| `http_server.c` | Body reading, JSON scraping, nine handlers, server configuration. |
| `www/index.html` | The dashboard — HTML, CSS and JS in one self-contained file. |
| `CMakeLists.txt` | Component registration and the embed. |

## See also

- [`tools/mock_server.py`](../../tools/mock_server.py) — reimplements every `/api/*` endpoint plus
  a crude flight sim, so `index.html` can be developed with no hardware. **If you change a
  response shape here, update it there too.**
- [`control_state`](../control_state/README.md) — where the button state actually becomes a
  setpoint.
