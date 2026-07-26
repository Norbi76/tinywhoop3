#pragma once

#include "esp_err.h"

// Dashboard HTTP server.
//
// Endpoints:
//   GET  /             the embedded dashboard (index.html, compiled into the binary)
//   POST /api/input    held-button state, 20 Hz from the browser
//   POST /api/arm      arm / disarm / kill
//   POST /api/mode     flight mode and the hold toggle
//   POST /api/capture  queue a photo - RETURNS IMMEDIATELY, never waits for the SD write
//   POST /api/gains    live PID gain update for one loop
//   GET  /api/status   telemetry readback for the dashboard
//
// Every handler is required to be fast. They run on the HTTP server task, and anything that
// blocks in here delays the 20 Hz input POSTs, which is what the control-link watchdogs on
// both boards are watching. Nothing in this file does file I/O or waits on a queue.

// Starts the HTTP server. wifi_ap_init() must have run first.
// @return ESP_OK on success, or an error code on failure.
esp_err_t http_server_start(void);

// Stops the server.
// @return ESP_OK on success, or an error code on failure.
esp_err_t http_server_stop(void);
