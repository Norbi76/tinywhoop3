#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

// Camera capture to SD card, for photogrammetry / 3D Gaussian Splatting reconstruction.
// Sensor model is detected at runtime; OV3660 (QXGA) and OV2640 (UXGA) are both supported.
//
// TWO THINGS SHAPE THIS MODULE:
//
// 1. EXPOSURE, WHITE BALANCE AND GAIN ARE LOCKED. Automatic exposure is actively harmful for
//    reconstruction: as the drone orbits an object, auto-exposure re-meters on every frame and
//    the same surface comes out a different brightness in every shot. Structure-from-motion
//    pipelines interpret that as a change in the scene rather than a change in the camera, and
//    the reconstruction degrades. Constant exposure across the whole capture set matters more
//    than any individual frame being correctly exposed.
//
// 2. SD WRITES NEVER TOUCH THE HTTP HANDLER OR THE CONTROL PATH. A FAT write to a cheap SD
//    card can block for hundreds of milliseconds when the card decides to do wear levelling.
//    If that happened inside the /api/capture handler it would stall the HTTP server, which
//    would stall the dashboard's 20 Hz input POSTs, which would trip the browser watchdog
//    mid-flight. So /api/capture only pushes a token onto a queue and returns; a dedicated
//    task pinned to core 1 does the actual grab-and-write, well away from the UART and HTTP
//    tasks on core 0.

// Running totals, surfaced on the dashboard so you can tell mid-flight whether shots are
// actually landing on the card.
typedef struct {
    uint32_t requested;      // capture requests accepted onto the queue
    uint32_t written;        // files successfully closed on the SD card
    uint32_t failed;         // grab failures, write failures, and requests dropped by a full queue
    uint32_t next_index;     // number the next file will get
    uint32_t last_size_bytes;
} camera_sd_stats_t;

// Initialises the camera and mounts the SD card.
//
// NEITHER IS FATAL. If the camera fails to probe, or there is no card in the slot, this logs
// the failure and returns an error, but the caller is expected to carry on booting - the drone
// must still fly with a broken camera. Check camera_sd_camera_ok() / camera_sd_sd_ok() to find
// out what actually came up.
//
// @return ESP_OK if both came up, ESP_ERR_NOT_FOUND if either did not.
esp_err_t camera_sd_init(void);

// Creates the capture task.
// @param core_id Core to pin to. Should be core 1, away from the UART and HTTP tasks.
// @return ESP_OK on success, ESP_FAIL if the task could not be created.
esp_err_t camera_sd_start_task(BaseType_t core_id);

// Queues one photo. RETURNS IMMEDIATELY - it does not wait for the grab or the SD write.
// @return ESP_OK if queued, ESP_ERR_INVALID_STATE if the camera or card is unavailable,
//         ESP_ERR_NO_MEM if the queue is full (the shot is dropped and counted as failed).
esp_err_t camera_sd_request_capture(void);

// Copies the running statistics out.
// @param out Destination, must not be NULL.
void camera_sd_get_stats(camera_sd_stats_t *out);

// ---------------------------------------------------------------------------
// LIVE PREVIEW (VIEWFINDER)
//
// Off by default, and it should stay off unless you are actually framing a shot: the radio is
// the expensive part of this board, and preview is the only thing that makes it transmit
// continuously.
//
// While preview is on the sensor drops to QVGA. It switches up to the capture resolution for
// the instant of each capture and re-applies the locked exposure, so shots taken with preview
// on are identical to shots taken with it off. Preview costs roughly 40x less data per frame
// than streaming the capture resolution would, which is what makes it viable at all.
// ---------------------------------------------------------------------------

// Turns the viewfinder on or off. RETURNS IMMEDIATELY - the sensor is owned by the capture
// task, and this only raises a flag that the task acts on. The HTTP handler never touches the
// sensor, so a slow frame grab can never delay the dashboard's 20 Hz control POSTs.
// @param enable true to start previewing, false to stop.
// @return ESP_OK, or ESP_ERR_INVALID_STATE if the camera never came up.
esp_err_t camera_sd_set_preview(bool enable);

// @return true if preview is currently requested.
bool camera_sd_preview_enabled(void);

// Copies the most recent preview JPEG out. Never blocks on the sensor - it copies from a
// buffer the capture task refreshes in the background, so it returns whatever the latest
// complete frame is.
// @param dst Destination buffer.
// @param dst_capacity Size of dst.
// @return Bytes copied, or 0 if no frame is available yet or dst is too small.
size_t camera_sd_copy_preview(uint8_t *dst, size_t dst_capacity);

// True if the camera initialised.
bool camera_sd_camera_ok(void);

// True if the SD card mounted.
bool camera_sd_sd_ok(void);
