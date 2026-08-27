# `wifi_ap` — SoftAP for the control link

Brings up the Wi-Fi access point the pilot's phone or laptop connects to. Small component, and
almost all of its value is in two settings that differ from the stock ESP-IDF SoftAP example.

**This is a control link, not a file server.** The drone is flown through it, so latency matters
far more than throughput. Every decision below follows from that.

## SoftAP, not station mode

The board is the access point; the pilot's device joins it. No router, no infrastructure, no
DHCP lease from someone else's network — connect and fly. Dashboard at the AP's own IP, printed
to the log at startup.

## The two settings that matter

### 1. Power save is OFF — this is why the component exists

```c
esp_wifi_set_ps(WIFI_PS_NONE);   // must be AFTER esp_wifi_start()
```

The ESP-IDF default is `WIFI_PS_MIN_MODEM`: the radio sleeps between beacons and buffers frames
until the next one. That adds **beacon-interval-sized gaps — up to ~100 ms — to every round trip**.

The dashboard posts control input at 20 Hz (every 50 ms). Adding up to 100 ms of jitter to each of
those is the difference between a responsive drone and an unflyable one.

The call **must** come after `esp_wifi_start()`; before it, it has no effect.

### 2. Transmit power is turned down

```c
#define WIFI_TX_POWER_QDBM 52   // units of 0.25 dBm → 13 dBm
```

The default is the radio's maximum, sized for tens of metres through walls. This drone is flown
across a room. Transmit power is the **single largest current draw the radio has**, and it matters
most exactly when the viewfinder is on and the radio is transmitting continuously.

**This is not the same trade as power save.** `WIFI_PS_NONE` keeps the radio *awake* so latency
stays low; TX power only changes how loudly it shouts. **Latency is unaffected** — the cost is
range and link margin, not responsiveness.

`TODO(bench)`: raise it if the dashboard stutters or the link drops at your actual flying range.
Units are 0.25 dBm; usable range is roughly 8 (2 dBm) to 80 (20 dBm). Try 60 (15 dBm) before going
higher.

Setting TX power is **non-fatal** if it fails — a louder radio than intended still flies, it just
costs more battery.

## ⚠ Credentials are a placeholder

```c
#define WIFI_AP_SSID     "TinyWhoopAP"
#define WIFI_AP_PASSWORD "whoop12345"
```

`TODO(config)`: **change these before flying anywhere with other people around.** WPA2 with a
known-weak password is only marginally better than an open network, and anyone on this AP can
reach `/api/arm`.

There is a deliberate fallback: if `WIFI_AP_PASSWORD` is shorter than 8 characters (WPA2's
minimum), the AP starts **open** rather than failing to start. An unreachable drone is worse than
an unsecured one — but it is logged as a warning, and it should not be how you fly.

Other settings: channel 1, max 2 connections (pilot + optionally the ground station), WPA2-PSK,
PMF not required.

## Startup sequence

```
nvs_flash_init()                  <- the Wi-Fi driver stores calibration data here
   └ on NO_FREE_PAGES / NEW_VERSION_FOUND: erase and retry
esp_netif_init()
esp_event_loop_create_default()   <- ESP_ERR_INVALID_STATE tolerated: already created
esp_netif_create_default_wifi_ap()
esp_wifi_init()
register the WIFI_EVENT handler
esp_wifi_set_mode(WIFI_MODE_AP) / set_config() / start()
esp_wifi_set_ps(WIFI_PS_NONE)     <- must be here, after start
esp_wifi_set_max_tx_power()
```

`wifi_ap_init()` is idempotent — returns `ESP_OK` immediately if already started.

## Event handler

Tracks the associated station count.

A **disconnect is logged at warning level**, not info: losing the station mid-flight is how the
control link dies, and it is worth being able to find that moment in the log afterwards.

`wifi_ap_get_client_count()` is surfaced on the dashboard so the pilot can spot a dropped link.

## Public API

| Function | Notes |
|---|---|
| `wifi_ap_init()` | The whole bring-up. Idempotent. |
| `wifi_ap_is_started()` | |
| `wifi_ap_get_client_count()` | Currently associated stations. |

## Timing and concurrency

- `wifi_ap_init()` is called once from `app_main()`, **after** the UART link (which is what
  actually flies the drone) and **before** the HTTP server and `gs_link` (both of which need the
  network up).
- The event handler runs on the ESP-IDF event task. `ap_started` and `client_count` are unlocked
  module statics; `client_count` is written only by that one task and read as a single `int`
  elsewhere, so a reader can see a stale value but never a torn one.
- The Wi-Fi/lwIP stack itself lives on core 0 alongside the HTTP server and both UART tasks.

## Dependencies

**Private** (`PRIV_REQUIRES`): `esp_wifi`, `esp_netif`, `esp_event`, `nvs_flash`. `wifi_ap.h`
exposes only `esp_err_t` and `bool`, so the entire Wi-Fi stack stays out of every caller's include
path.

## Files

| File | Contents |
|---|---|
| `include/wifi_ap.h` | Credentials, channel, TX power, three public functions. |
| `wifi_ap.c` | Event handler, the full bring-up sequence, the two settings above. |
| `CMakeLists.txt` | Component registration. |
