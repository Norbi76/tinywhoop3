# `uart_link` — the serial link to the flight controller

Two FreeRTOS tasks that carry everything between this board and the drone's flight controller.
This is the component that actually flies the drone; the Wi-Fi, the dashboard and the camera are
all downstream of it.

The wire protocol lives in
[`shared_components/telemetry_uart`](../../../shared_components/telemetry_uart/README.md).

## Two tasks, pinned to core 0

| Task | Priority | Cadence | Job |
|---|---:|---|---|
| `UART_TX` | 7 | **Fixed 50 Hz** | Integrate setpoints, send a control frame, piggyback queued gains and ground-station uplink |
| `UART_RX` | 6 | Continuous, 5 ms wakeups | Drain frames, cache the status, forward everything to the ground station |

**TX sits above RX in priority.** Missing a transmit slot risks the flight controller's 300 ms
link watchdog disarming the drone; a late status frame only makes the dashboard slightly stale.

Both go on core 0 alongside Wi-Fi and the HTTP server, so the 50 Hz TX cadence and the 20 Hz
dashboard POSTs are scheduled against each other by one scheduler with no cross-core latency.
Core 1 is left entirely to the camera.

## TX: fixed rate, no matter what

The control frame goes out **50 times a second whether or not the pilot is touching anything**.

A "send on change" design would look more efficient and would disarm the drone the moment the
pilot stopped pressing buttons — the flight controller cannot distinguish "nothing to say" from
"the link is dead", so silence *is* the failure signal.

Per cycle, in order:

1. `control_state_update(dt)` — advance the ramp/decay integration and run the browser watchdog.
   This lives here, not in the HTTP handler, so setpoints evolve at a fixed known rate even when
   the browser posts irregularly. See [`control_state`](../control_state/README.md).
2. `control_state_get_frame()` → send `TELEMETRY_MSG_CONTROL`. **This is the important one.**
3. Up to `GAINS_PER_CYCLE` (2) queued dashboard gain updates.
4. Up to `UPLINK_PER_CYCLE` (4) queued ground-station frames.

Both piggyback limits are small on purpose: a burst of slider drags or a tuning session must never
push the control frame late. At 50 Hz they still clear 100 gain updates and 200 uplink frames per
second, which is far more than either source produces.

`vTaskDelayUntil()`, not `vTaskDelay()` — the cadence must not drift with however long the sends
took.

Send failures are logged **at most once per second**. At 50 Hz an unconditional log would
saturate the console UART on its own; the uplink relay is rate-limited the same way.

## RX: bounded drain, then one datagram

```c
while (drained < RX_DRAIN_PER_CYCLE) {          // 64
    if (!uart_telemetry_read_message_resync(&message, 2)) break;
    ... handle ...
    gs_link_forward(&message);
    drained++;
}
gs_link_flush();
vTaskDelay(5 ms);
```

**Why drain in a batch rather than one frame per wakeup.** With PID debug frames arriving at up to
500 Hz on top of the 50 Hz status stream, a one-frame-per-cycle reader would fall permanently
behind and the dashboard's attitude readout would age without bound. 64 frames per 5 ms wakeup is
12 800 frames/second of headroom against a worst case near 550.

**Why the resync reader.** `uart_telemetry_read_message_resync()` discards only the bytes it
consumed on a bad frame instead of flushing the whole RX buffer. A flush would take good frames
with it — and this is a link where sustained loss trips a watchdog.

**Why one datagram per drain, not per frame.** `gs_link_forward()` only appends to a batch;
`gs_link_flush()` sends it. At 500 Hz, one datagram per frame would be 500 packets/second and the
SoftAP would choke on the packet *rate* long before the bitrate.

### Frame handling

| Message | Action |
|---|---|
| `TELEMETRY_MSG_STATUS` | Length-checked, then `control_state_store_status()`, `rx_frame_count++` |
| `TELEMETRY_MSG_IMU` | Nothing — the dashboard reads attitude from the status frame. Handled explicitly so it isn't treated as junk. |
| everything else | Nothing locally |

**Every frame is forwarded to the ground station, including the ones the switch ignores.** The
dashboard and the ground station are independent consumers and neither filters for the other.

## The gains queue

`uart_link_queue_gains()` is called from the **HTTP handler** and uses a **zero timeout** — it
must never block a web request. A full queue (depth 8) logs a warning and returns
`ESP_ERR_NO_MEM`; the pilot moves a slider again.

## Why the uplink queue lives in `gs_link`, not here

`gs_link`'s UDP task cannot call `uart_telemetry_send_message()` directly, because that function
is **not thread-safe** — it bumps a shared sequence counter and issues three separate
`uart_write_bytes()` calls, so a concurrent caller interleaves its bytes into another task's frame
and produces CRC failures on the link that flies the drone.

So the UDP task queues, and this TX task drains — keeping it the single writer, exactly as the
dashboard's gain updates already do.

The queue is placed **inside `gs_link`** so the dependency runs one way: `uart_link` privately
requires `gs_link`, and `gs_link` knows nothing about `uart_link`. Putting the queue here would
have created a cycle.

## ⚠ Pins are unconfirmed — and there is history here

```c
#define UART_LINK_TX_PIN 43
#define UART_LINK_RX_PIN 44
#define UART_LINK_BAUD_RATE 460800
#define UART_LINK_PORT UART_NUM_1
```

**These are not the pins the old `main.c` used.** The previous code used TX=GPIO9, RX=GPIO8 — and
on the XIAO ESP32-S3 Sense those are the SD card's CMD and DATA0 lines on the expansion board
(CLK=7, CMD=9, D0=8). Sharing them with the telemetry UART cannot work: whichever peripheral
initialises second wins, and the symptom is either a dead link or an SD card that never mounts.

GPIO 43/44 are the XIAO's D6/D7 pads, the chip's default UART0 TX/RX. They are free for UART1 use
here as long as you are not depending on a USB-serial console on those pads — the XIAO's console
runs over native USB, so this is normally fine.

Still marked `TODO(pins)`. Check against your actual harness before flashing.

> The flight controller's end of this link is configured for GPIO 6/7 in
> `flight_controller/main/main.c`, also marked TODO. Different boards, so the numbers need not
> match — but both must match the wiring.

## Public API

| Function | Notes |
|---|---|
| `uart_link_init()` | Creates the gains queue and initialises the UART driver. |
| `uart_link_start(core_id)` | Creates both tasks. Should be core 0. |
| `uart_link_queue_gains(&g)` | Non-blocking. Frame goes out within 20 ms. |
| `uart_link_get_rx_count()` | Valid status frames since boot — diagnostics. |
| `uart_link_get_tx_count()` | TX cycles since boot. |

## Dependencies

**Public** (`REQUIRES`): `telemetry_uart` — `telemetry_gains_payload_t` is in `uart_link.h`.
**Private** (`PRIV_REQUIRES`): `control_state`, `gs_link`.

## Files

| File | Contents |
|---|---|
| `include/uart_link.h` | Pins, baud, TX rate, five public functions, the pin-history warning. |
| `uart_link.c` | Both tasks, the piggyback limits, the drain loop, the gains queue. |
| `CMakeLists.txt` | Component registration with the one-way `gs_link` dependency explained. |
