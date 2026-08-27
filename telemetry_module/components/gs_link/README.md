# `gs_link` — UDP bridge to the Python ground station

Tunnels the flight controller's PID tuning traffic over Wi-Fi. Frames arriving on the UART are
batched into UDP datagrams and sent to the ground station; datagrams coming back are split into
records and queued for the UART TX task to relay to the drone.

The telemetry module **does not interpret** any of this traffic. It is a pipe.

```
flight_controller  ──UART──>  uart_link RX  ──>  gs_link  ──UDP:14550──>  tools/ground_station
flight_controller  <──UART──  uart_link TX  <──  gs_link  <──UDP────────  tools/ground_station
```

It runs alongside the HTTP dashboard and is deliberately independent of it: the dashboard keeps
getting its status frames whether or not a ground station is connected, and the ground station
works whether or not a browser is open.

## Wire format — not the UART frame

One datagram carries **N concatenated records**:

```
  msg_type(1) | payload_len(1) | seq(2, little-endian) | payload(payload_len)
```

No start byte, no CRC, no end byte.

The UART frame needs all three because a serial line has no framing of its own and no error
detection. UDP already provides both — a datagram either arrives intact or does not arrive.
Re-wrapping the UART frame would spend roughly 20 % of the link on redundant bytes.

The ESP32-S3 is little-endian, so the 16-bit `seq` is a straight `memcpy` and already matches the
little-endian field the Python side unpacks.

A **zero-length record** is the ground station's keepalive. It exists purely to keep the peer
address fresh (below); there is nothing to forward.

## Peer discovery — the link is self-healing

There is no configured ground-station address. **Any inbound packet latches its sender as the
current peer**, before the contents are even examined.

That is what lets you restart the ground station, or let it pick up a new IP after a Wi-Fi drop,
and have telemetry follow it without touching the drone.

Until something has been received, outbound datagrams are **dropped silently**. That is the normal
state whenever nobody is tuning — not an error, and deliberately not logged.

The peer change is logged **only on an actual change**, not per packet: the ground station sends a
keepalive every second, and logging every one would be a permanent once-a-second console line.

## Batching

`gs_link_forward()` appends to a buffer. `gs_link_flush()` sends it and clears it.

`GS_LINK_BATCH_SIZE` is **1200 bytes** — comfortably under a 1500-byte Ethernet MTU once the UDP
and IP headers (28 bytes) are on, so nothing ever fragments. That holds 31 debug records.

If a record would not fit, `forward()` flushes first and then appends. A record that could *never*
fit is dropped outright rather than looping on an empty batch.

The batch is touched only by `forward()` and `flush()`, both called from the UART RX task, so it
needs no lock of its own. The **peer** does need one — it is written by this component's receive
task and read by `flush()` on the UART RX task, and a torn `sockaddr_in` would send telemetry to
an address that never existed.

## The uplink is a queue, not a direct send

This is the single most important design decision in the component.

`uart_telemetry_send_message()` is **not thread-safe**. It bumps a shared sequence counter and
then issues three separate `uart_write_bytes()` calls. A second task calling it concurrently
interleaves its bytes into another task's frame, producing CRC failures **on the link that flies
the drone**.

So the UDP receive task queues (`gs_link_handle_datagram()` → `xQueueSend`), and `uart_link`'s TX
task drains (`gs_link_pop_uplink()`), keeping that task the single UART writer — exactly as the
dashboard's gain updates already do.

**The queue lives in this component rather than in `uart_link` so the dependency runs one way:**
`uart_link` privately requires `gs_link`; `gs_link` knows nothing about `uart_link`. Putting the
queue in `uart_link` would have created a cycle.

`GS_LINK_UPLINK_QUEUE_LENGTH` is 16. A tuning session sends bursts — switching a tab produces a
select, a gain read-back and sometimes an inject in one datagram — so this holds several bursts
while the 50 Hz TX task works through them. Items are carried **by value** so the receive task's
datagram buffer is reusable the instant `recvfrom()` returns.

The queue send uses a **zero timeout**: a stalled TX task must not back-pressure into the UDP
task. A dropped uplink frame costs the operator one retry, which is the right trade.

## Priority and failure handling

`GS_LINK_TASK_PRIORITY` is **4** — below both UART tasks (7 and 6). A ground station that has
stopped reading must never delay a control frame, and losing tuning data is always preferable to
losing the link that flies the drone. Pinned to core 0.

The task blocks on `recvfrom()`, which is why it is a task rather than a poll.

`sendto()`'s return value in `flush()` is **deliberately ignored**. This is telemetry over UDP on
a link that may well be down; a failed send is not worth reporting at this rate, and logging it
would put a line on the console for every flush.

A truncated record stops parsing for the rest of the datagram rather than guessing where the next
record begins.

## Public API

| Function | Called from | Notes |
|---|---|---|
| `gs_link_start()` | `app_main`, **after** the Wi-Fi AP is up | Creates the mutex, queue, socket, and task. Binds UDP `GS_LINK_PORT` (14550). |
| `gs_link_forward(&msg)` | `uart_link` RX task | Appends to the batch; does not transmit. Safe to call for every frame. |
| `gs_link_flush()` | `uart_link` RX task | Transmits and clears. Once per drain. |
| `gs_link_pop_uplink(&type, buf, &len)` | `uart_link` TX task | `buf` must be at least `TELEMETRY_MAX_PAYLOAD_SIZE`. |

## Dependencies

**Public** (`REQUIRES`): `telemetry_uart` — `gs_link_forward()` takes a `telemetry_message_t`.
**Private** (`PRIV_REQUIRES`): `lwip` — the socket is entirely an implementation detail.

Note this component deliberately does **not** require `uart_link`; see the uplink-queue section.

## Files

| File | Contents |
|---|---|
| `include/gs_link.h` | Port, four public functions, the wire format and the queue rationale. |
| `gs_link.c` | Socket setup, peer latching, datagram parsing, the outbound batch. |
| `CMakeLists.txt` | Component registration with the one-way dependency explained. |

## See also

- [`tools/ground_station/README.md`](../../../tools/ground_station/README.md) — the other end.
- [`shared_components/telemetry_uart`](../../../shared_components/telemetry_uart/README.md) — the
  `PID_*` message block that rides this bridge.
