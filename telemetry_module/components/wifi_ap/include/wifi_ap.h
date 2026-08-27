#pragma once

#include <stdbool.h>
#include "esp_err.h"

// wifi_ap.h - the SoftAP the pilot's phone or laptop connects to.
//
// WHAT THIS COMPONENT IS FOR
//   Brings up NVS, netif, the event loop and a WPA2 SoftAP, then applies the two settings that
//   make it a CONTROL link rather than a general-purpose one. Almost all of this component's
//   value is in those two lines - the rest is the stock ESP-IDF bring-up sequence.
//
// HOW IT DOES ITS JOB - the two settings, and why they are different trades
//   esp_wifi_set_ps(WIFI_PS_NONE)  keeps the radio awake. The default WIFI_PS_MIN_MODEM buffers
//     frames until the next beacon, adding up to ~100 ms of jitter to a 50 ms control POST.
//     Costs current, buys latency. MUST be called after esp_wifi_start().
//   esp_wifi_set_max_tx_power()    turns the radio DOWN from its default maximum. TX power is the
//     radio's largest current draw and this drone flies across a room. Costs range, and does NOT
//     affect latency - it is an independent knob from power save.
//
// SoftAP for the control dashboard.
//
// This is a control link, not a file server: the drone is flown through it, so latency
// matters far more than throughput. Wi-Fi power save is therefore switched OFF, which costs
// current but removes the beacon-interval-sized gaps (up to ~100 ms) that the modem would
// otherwise introduce into every round trip.

// Default AP credentials.
// Wi-Fi transmit power, in units of 0.25 dBm. 52 = 13 dBm, which is ample for flying across a
// room and noticeably cheaper on the battery than the ~20 dBm default. Raise it if the link
// stutters at your actual flying range; see the note in wifi_ap.c.
#define WIFI_TX_POWER_QDBM 52

// TODO(config): change these before flying anywhere with other people around. WPA2 with a
// known-weak password is only marginally better than an open network.
#define WIFI_AP_SSID "TinyWhoopAP"
#define WIFI_AP_PASSWORD "whoop12345"   // must be >= 8 characters for WPA2
#define WIFI_AP_CHANNEL 1
#define WIFI_AP_MAX_CONNECTIONS 2

// Brings up NVS, the netif stack, the event loop and the SoftAP, then disables power save.
// @return ESP_OK on success, or an error code on failure.
esp_err_t wifi_ap_init(void);

// True once the AP is running.
bool wifi_ap_is_started(void);

// Number of stations currently associated. Useful on the dashboard to spot a dropped link.
int wifi_ap_get_client_count(void);
