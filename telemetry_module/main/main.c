// main.c - the telemetry module's entry point. app_main() and nothing else.
//
// WHAT THIS FILE DOES
//   Every subsystem lives in a component; this file's only job is to bring them up in the right
//   ORDER and put them on the right CORE. Both are load-bearing, so both are documented below.
//
// HOW THE ORDER IS DECIDED
//   Dependency first, priority second: control_state (depends on nothing) -> UART link (the thing
//   that actually flies the drone, before Wi-Fi) -> camera/SD -> Wi-Fi AP -> gs_link (needs a
//   netif) -> HTTP server LAST, because it is what starts accepting pilot input and everything it
//   touches must already exist.
//
// FATAL vs NON-FATAL IS A DELIBERATE SPLIT
//   Fatal:     control_state, uart_link, wifi_ap, http_server  - the pilot cannot fly without them.
//   Non-fatal: camera_sd, gs_link                              - they only make the drone more useful.
//   The flight controller applies the same principle to its ToF and optical flow sensors.
//
// Core assignment and the priority ladder are explained in the comment block below.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "wifi_ap.h"
#include "control_state.h"
#include "uart_link.h"
#include "camera_sd.h"
#include "http_server.h"
#include "gs_link.h"

static const char *TAG = "TELEMETRY_MODULE";

// CORE ASSIGNMENT
//
// Core 0: Wi-Fi and lwIP (which the IDF pins there anyway), the HTTP server, and both UART
//         tasks. Keeping the whole control path on one core means the 50 Hz transmit cadence
//         and the 20 Hz input POSTs are scheduled against each other by one scheduler, with
//         no cross-core latency in between.
//
// Core 1: the camera capture task, alone. Grabbing a UXGA JPEG out of PSRAM and pushing it
//         through FATFS onto an SD card is a long, bursty, blocking job. Isolating it on the
//         other core means a slow card cannot delay a control frame no matter how long it
//         takes.
#define CORE_CONTROL 0
#define CORE_CAMERA 1

void app_main(void) {
    ESP_LOGI(TAG, "Telemetry module starting up...");

    esp_err_t error;

    // --- 1. Control state ----------------------------------------------------
    // First, so that every later module has somewhere valid to read from and write to.
    error = control_state_init();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Control state init failed: %s", esp_err_to_name(error));
        return;
    }

    // --- 2. UART link to the flight controller -------------------------------
    // Before Wi-Fi: this is the link that actually flies the drone, and if it cannot come up
    // there is no point serving a dashboard.
    error = uart_link_init();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "UART link init failed: %s", esp_err_to_name(error));
        return;
    }

    error = uart_link_start(CORE_CONTROL);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start UART tasks: %s", esp_err_to_name(error));
        return;
    }

    // --- 3. Camera and SD card -----------------------------------------------
    // NON-FATAL BY DESIGN. A missing SD card or a failed camera probe must not stop the drone
    // flying - photography is the mission, but flight is the prerequisite. Errors here are
    // logged, surfaced on the dashboard, and otherwise ignored.
    error = camera_sd_init();
    if (error != ESP_OK) {
        ESP_LOGW(TAG, "Camera/SD not fully available (%s) - continuing, the drone can still fly",
                 esp_err_to_name(error));
    }

    // Started even if init partly failed: camera_sd_request_capture() rejects requests on its
    // own when the hardware is missing, and having the task running keeps the shutdown paths
    // uniform.
    error = camera_sd_start_task(CORE_CAMERA);
    if (error != ESP_OK) {
        ESP_LOGW(TAG, "Failed to start capture task: %s", esp_err_to_name(error));
    }

    // --- 4. Wi-Fi SoftAP -----------------------------------------------------
    error = wifi_ap_init();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi AP init failed: %s", esp_err_to_name(error));
        return;
    }

    // --- 5. Ground station link ----------------------------------------------
    // NON-FATAL, same reasoning as the camera: this is a tuning tool, not a flight requirement.
    // Started after the AP so lwIP has a netif to bind to, and before the HTTP server only
    // because the HTTP server is the thing that starts accepting pilot input.
    error = gs_link_start();
    if (error != ESP_OK) {
        ESP_LOGW(TAG, "Ground station link unavailable (%s) - PID tuning over UDP is disabled",
                 esp_err_to_name(error));
    }

    // --- 6. HTTP server ------------------------------------------------------
    // Last, because it is the thing that starts accepting pilot input, and everything it
    // touches has to exist before the first request arrives.
    error = http_server_start();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server failed to start: %s", esp_err_to_name(error));
        return;
    }

    ESP_LOGI(TAG, "Startup complete.");
    ESP_LOGI(TAG, "  Connect to Wi-Fi '%s' and open http://192.168.4.1/", WIFI_AP_SSID);
    ESP_LOGI(TAG, "  Camera: %s   SD card: %s",
             camera_sd_camera_ok() ? "OK" : "UNAVAILABLE",
             camera_sd_sd_ok() ? "OK" : "UNAVAILABLE");
}
