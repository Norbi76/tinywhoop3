#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "wifi_ap.h"
#include "control_state.h"
#include "uart_link.h"
#include "camera_sd.h"
#include "http_server.h"

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

    // --- 5. HTTP server ------------------------------------------------------
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
