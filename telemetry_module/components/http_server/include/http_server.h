#pragma once

#include "esp_err.h"

// http_server.h - the dashboard and the "/api/..." control surface.
//
// WHAT THIS COMPONENT IS FOR
//   Everything the pilot does - arm, fly, change mode, take a photo, tune a gain - arrives as an
//   HTTP request here. The component holds NO state of its own: each handler translates a request
//   into a call on control_state, camera_sd or uart_link and returns.
//
// HOW IT DOES ITS JOB
//   Every write handler is a QUEUE PUSH, never the work itself. /api/capture queues a token
//   instead of writing to the SD card; /api/gains queues a frame instead of sending it;
//   /api/input records button state and lets the 50 Hz UART task do the integration. That
//   pattern is the whole design - see the "must be fast" note below for why.
//
//   Request bodies are parsed by hand (json_flag/json_number in the .c) rather than with a JSON
//   library, because they are three or four flat keys from a single known producer.
//
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
