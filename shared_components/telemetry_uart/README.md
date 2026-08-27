# `telemetry_uart` — the two-board wire protocol

The framed binary UART protocol spoken between the **flight controller** board and the
**telemetry module** board, plus the framing/CRC code that implements it. This is the only
component in the repo that is shared: it lives once on disk here and is pulled into *both*
firmware projects through `EXTRA_COMPONENT_DIRS` in each project's top-level `CMakeLists.txt`.
It is **not vendored** — there is no second copy to keep in sync, which also means any edit here
changes both firmwares at once.

```
flight_controller  <──── UART ────>  telemetry_module  <──── UDP ────>  tools/ground_station
   (this protocol)                     (this protocol)                    (same frames, tunnelled)
```

## What it does

1. Defines the **frame format** — start byte, message type, payload length, sequence number,
   payload, CRC16, end byte.
2. Defines the **message types** and the **payload structs** for each one. These structs are the
   actual contract: both boards `memcpy` the same packed layout in and out of frames, so a field
   added on one side and not the other silently mis-decodes on the wire.
3. Provides four functions: `uart_telemetry_init()`, `uart_telemetry_send_message()`,
   `uart_telemetry_read_message()`, `uart_telemetry_read_message_resync()`, plus the CRC helper.

## Frame layout

```
  offset  size  field
  ------  ----  --------------------------------------------------
     0     1    start_byte     always 0xAA
     1     1    msg_type       telemetry_msg_type_t
     2     1    payload_len    0 .. TELEMETRY_MAX_PAYLOAD_SIZE (64)
     3     2    seq            free-running counter, little endian
     5     N    payload        N == payload_len
   5+N     2    crc16          CRC-16/CCITT-FALSE over header+payload
   7+N     1    end_byte       always 0xBB
```

Total overhead is 8 bytes per frame. `crc16` covers the header **and** the payload — including
`start_byte` — so a corrupted length or type byte is caught, not just corrupted payload data.

The CRC is CRC-16/CCITT-FALSE: init `0xFFFF`, polynomial `0x1021`, MSB-first, no reflection, no
final XOR. `tools/ground_station/protocol.py` reimplements exactly this in Python; if you change
the polynomial or the init value here, that file has to change with it.

## Message types

| Value | Name | Direction | Payload |
|------:|------|-----------|---------|
| 1  | `TELEMETRY_MSG_IMU`        | FC → TM | `telemetry_imu_payload_t` |
| 2  | `TELEMETRY_MSG_BATTERY`    | FC → TM | `telemetry_battery_payload_t` |
| 3  | `TELEMETRY_MSG_STATUS`     | FC → TM | `telemetry_status_payload_t` |
| 10 | `TELEMETRY_MSG_CONTROL`    | TM → FC | `telemetry_control_payload_t` |
| 11 | `TELEMETRY_MSG_ARMING`     | TM → FC | (reserved; arming rides in the control frame) |
| 12 | `TELEMETRY_MSG_MODE`       | TM → FC | (reserved; mode rides in the control frame) |
| 13 | `TELEMETRY_MSG_GAINS`      | TM → FC | `telemetry_gains_payload_t` |
| 20 | `TELEMETRY_MSG_PID_DEBUG`  | FC → GS | `telemetry_pid_debug_payload_t` |
| 21 | `TELEMETRY_MSG_PID_GAINS`  | both    | `telemetry_pid_gains_payload_t` |
| 22 | `TELEMETRY_MSG_PID_SELECT` | GS → FC | `telemetry_pid_select_payload_t` |
| 23 | `TELEMETRY_MSG_PID_INJECT` | GS → FC | `telemetry_pid_inject_payload_t` |

"GS" is the Python ground station in `tools/ground_station`. The `20..23` block is **tunnelled**:
the telemetry module does not interpret those frames at all, it forwards them verbatim between the
UART and a UDP socket (see `telemetry_module/components/gs_link`). That is why they are defined
here rather than in a ground-station-only header — the flight controller and the Python client
need the identical struct layout, and the telemetry module needs the type numbers to know what to
forward.

## The two independent PID loop numberings

There are **two** enums addressing the same eight PID instances, and this is deliberate:

- `telemetry_loop_id_t` — used by `TELEMETRY_MSG_GAINS` (the web dashboard). Ordered
  inner-to-outer: rate roll/pitch/yaw, angle roll/pitch, velocity x/y, altitude.
- `pid_loop_id_t` — used by the `PID_*` message block (the Python ground station). Ordered
  outer-to-inner: altitude, velocity x/y, angle roll/pitch, rate roll/pitch/yaw.

They were not unified because the dashboard's loop ids are baked into `index.html` as literal
numbers; renumbering `telemetry_loop_id_t` would silently retune the wrong axis from the browser.
`pid_registry_bind()` on the flight controller reconciles them — each PID instance keeps its
`telemetry_loop_id_t` array slot and is additionally *bound* to its `pid_loop_id_t`.

If you touch either enum, both sides break: grep for `telemetry_loop_id_t` (dashboard +
`flight_control.c`) and `pid_loop_id_t` (`pid_registry.c` + `tools/ground_station/protocol.py`).

## Compile-time size guards

The header ends in a block of `_Static_assert`s. There are two kinds and they exist for different
reasons:

- **Fits-in-a-frame** (`<= TELEMETRY_MAX_PAYLOAD_SIZE`) — without these, adding a field to a
  payload struct would overflow `telemetry_message_t.payload` at runtime and corrupt the frame
  tail. That presents as intermittent CRC failures, not as an obvious bug.
- **Exact size** (`== 30`, `== 25`, `== 4`, `== 10`, on the PID payloads only) — the Python ground
  station unpacks those with hardcoded `struct` formats (`"<IBB6f"` and friends). A compiler-
  inserted padding byte would still fit the frame and would still build; it would just shift every
  float in the plot by one byte and produce something that reads as noise rather than as a bug.

`TELEMETRY_MAX_PAYLOAD_SIZE` is 64 (raised from 48 so the 53-byte status frame fits).
`TELEMETRY_PID_PAYLOAD_CEILING` stays at 48 because the ground station's buffers are sized against
the old limit — the PID payloads are asserted against the tighter of the two.

## The two readers, and why you want `_resync()`

`uart_telemetry_read_message()` is the straightforward one: read header, validate, read payload,
read tail, check CRC. On **any** validation failure it calls `uart_flush_input()`.

That flush is the problem. It throws away not just the bad bytes but every good frame already
queued behind them. On a link carrying steady 50 Hz traffic, one corrupted byte therefore costs
several frames, and the flight controller's 300 ms control-link watchdog is only ~15 frames wide.

`uart_telemetry_read_message_resync()` never flushes. It scans forward one byte at a time looking
for `0xAA`, and on a bad length / bad end byte / CRC mismatch it simply continues scanning from
wherever it stopped. Worst case it loses the one frame it was mid-way through. It takes a total
time budget in milliseconds so a caller in a periodic task can bound how long it blocks.

**Use `_resync()` on both boards' RX paths.** The plain reader is kept because it is simpler and
is fine on a link with bursty request/response traffic, but nothing on this airframe has that
shape.

## Timing and concurrency

- `uart_telemetry_init()` installs the ESP-IDF UART driver with a 2 KB TX and 2 KB RX ring buffer
  and stores the port in a **module-global** (`active_uart_num`). Consequence: this component
  supports exactly **one** telemetry UART per firmware image. Both projects only need one.
- `sequence_counter` is also a plain module-global, incremented in `uart_telemetry_send_message()`
  with no lock. **All sends must come from one task.** Both firmwares comply (the flight
  controller sends only from `telemetry_task`, the telemetry module only from its UART TX task) —
  if you ever add a second sending task, the sequence numbers will interleave and the
  send-side CRC scratch buffer becomes a race.
- Reads and writes use the ESP-IDF driver's own ring buffers, so TX does not block on the wire as
  long as the buffer has room.
- Per-read timeouts inside the frame are hardcoded at 2 ms — long enough for a full 64-byte
  payload at the baud rates in use, short enough that a truncated frame does not stall a 50 Hz
  task.

## Public API

| Function | Purpose |
|---|---|
| `uart_telemetry_init(cfg)` | Configure port/pins/baud, install the driver. Call once. |
| `uart_telemetry_send_message(type, payload, len)` | Frame and write one message. |
| `uart_telemetry_read_message(out)` | Read one frame; flushes RX on error. |
| `uart_telemetry_read_message_resync(out, timeout_ms)` | Read one frame; resyncs instead of flushing. Preferred. |
| `telemetry_calculate_crc16(data, len)` | CRC-16/CCITT-FALSE. Exposed so callers can verify a frame they built themselves. |

Pins, port and baud rate are **not** defined here — each firmware passes its own
`telemetry_uart_config_t`. See `flight_controller/main/main.c` and
`telemetry_module/components/uart_link/include/uart_link.h`. Both are currently marked
`TODO(pins)` pending wiring confirmation, and they do not presently agree; confirm against the
actual harness before flashing.

## Dependencies

`esp_driver_uart`, `esp_driver_gpio`, `esp_timer` (the resync reader's deadline). No FreeRTOS
objects of its own.

## Files

| File | Contents |
|---|---|
| `include/telemetry_uart.h` | The entire contract: frame structs, message enum, payload structs, loop-id enums, flag bits, static asserts, function declarations. |
| `telemetry_uart.c` | CRC, driver init, framing on send, the two readers. |
| `CMakeLists.txt` | Component registration. |

## Changing this component

Any change ripples to three codebases. Before editing:

1. Read the "shared protocol" sections of both `flight_controller/CLAUDE.md` and
   `telemetry_module/CLAUDE.md`.
2. Grep the *other* firmware for the other side of your change.
3. If you touched a `PID_*` payload or either loop-id enum, update
   `tools/ground_station/protocol.py` too — it has its own hand-written copy of these layouts and
   the exact-size asserts here are the only thing that will catch a drift.
