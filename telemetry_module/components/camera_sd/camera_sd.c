#include "camera_sd.h"
#include "esp_camera.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include "esp_heap_caps.h"
#include "freertos/semphr.h"

static const char *TAG = "CAMERA_SD";

// ---------------------------------------------------------------------------
// XIAO ESP32-S3 Sense camera pins.
//
// Unlike the flight controller's pins, these are NOT free choices - the camera is wired to
// the XIAO Sense's board-to-board connector and these values are fixed by Seeed's design.
// They are reproduced from Seeed's published pin configuration. Worth a sanity check against
// the wiki page for your board revision, but do not "correct" them to match a different
// ESP32-S3 camera board (the AI-Thinker CAM uses a completely different mapping).
// ---------------------------------------------------------------------------
#define CAM_PIN_PWDN  -1   // not connected on the XIAO Sense
#define CAM_PIN_RESET -1   // not connected; the sensor is reset over SCCB
#define CAM_PIN_XCLK  10
#define CAM_PIN_SIOD  40   // SCCB data
#define CAM_PIN_SIOC  39   // SCCB clock
#define CAM_PIN_D7    48
#define CAM_PIN_D6    11
#define CAM_PIN_D5    12
#define CAM_PIN_D4    14
#define CAM_PIN_D3    16
#define CAM_PIN_D2    18
#define CAM_PIN_D1    17
#define CAM_PIN_D0    15
#define CAM_PIN_VSYNC 38
#define CAM_PIN_HREF  47
#define CAM_PIN_PCLK  13

// ---------------------------------------------------------------------------
// SD card pins (XIAO ESP32-S3 Sense expansion board, SDMMC in 1-bit mode).
//
// >>> THESE COLLIDE WITH THE OLD TELEMETRY UART PINS. <<<
// The previous main.c used GPIO9 for UART TX and GPIO8 for UART RX. Those are this card's
// CMD and DATA0. The UART has been moved to GPIO 43/44 in uart_link.h - see the note there.
// ---------------------------------------------------------------------------
#define SD_PIN_CLK 7
#define SD_PIN_CMD 9
#define SD_PIN_D0  8

#define SD_MOUNT_POINT "/sdcard"

// ---------------------------------------------------------------------------
// Exposure calibration.
//
// Exposure and gain are no longer hard-coded constants. camera_sd_calibrate_exposure() runs
// the sensor's own auto-exposure against your actual scene at boot, reads back what it
// converged on, and locks the sensor there for the rest of the session. That gives the
// constant-exposure property a capture set needs WITHOUT depending on a number guessed at
// compile time against unknown lighting.
//
// Point the drone at a representative part of the scene while it boots.
// ---------------------------------------------------------------------------

// Frames to discard while auto-exposure converges. The OV3660 needs several; 8 is comfortably
// past the knee and still under a second and a half of boot time.
#define CAM_AEC_CALIBRATION_FRAMES 8

// Pause between calibration frames, giving the AEC loop time to react to the previous one.
#define CAM_AEC_SETTLE_MS 60

// ---------------------------------------------------------------------------
// TODO(bench): CAM_AE_LEVEL - exposure bias applied while calibrating, range -2..2.
//
// This is the one exposure knob left to turn, and it is a relative bias rather than an
// absolute guess, so it stays meaningful across different rooms.
//
//   -1  slightly darker than the sensor's own choice (the default here)
//    0  exactly what the sensor picked
//   -2  darker still, for bright rooms or if you see blown highlights
//
// Biased dark on purpose: a clipped highlight is unrecoverable and contributes no features,
// while a slightly dark frame still carries the detail. A shorter exposure also means less
// motion blur, which matters on a moving drone - blur corrupts feature POSITIONS, which is
// worse for reconstruction than the noise you get from underexposure.
// ---------------------------------------------------------------------------
#define CAM_AE_LEVEL -1

// ---------------------------------------------------------------------------
// TODO(bench): CAM_WB_MODE - pick the preset that matches your lighting.
//
// Fixed white balance preset. Any value 1..4 puts the OV3660's AWB into manual mode
// (register 0x3406 = 1) and loads constant R/G/B gains, so colour is IDENTICAL on every shot -
// which is what photogrammetry needs, and what mode 0 does NOT give you.
//
//   1  Sunny      daylight through a window
//   2  Cloudy     overcast daylight
//   3  Office     fluorescent / most white LED lighting   <-- default, the usual indoor case
//   4  Home       incandescent / warm white bulbs
//   0  Auto       DO NOT USE for a capture set: colour then drifts between shots
//
// How to choose: take one test shot under your actual lighting with each of 3 and 4 and keep
// whichever renders a white wall closest to neutral. Getting this slightly wrong costs you a
// mild uniform colour shift, which is harmless for reconstruction because it is the same in
// every frame. Leaving it on 0 costs you inconsistency between frames, which is not harmless.
// ---------------------------------------------------------------------------
#define CAM_WB_MODE 3

// JPEG quality, 10..63 where LOWER is better. 10-12 is right for photogrammetry; compression
// artefacts look like texture to a feature detector and produce spurious matches.
#define CAM_JPEG_QUALITY 10

// Two frame buffers so a grab can start while the previous one is still being written.
// Both live in PSRAM - a QXGA buffer will not fit in internal SRAM.
#define CAM_FB_COUNT 2

// ---------------------------------------------------------------------------
// Live preview (viewfinder).
//
// You cannot frame a shot without seeing through the lens, but streaming the capture
// resolution is not an option: a QXGA JPEG at q10 is a few hundred kilobytes, so even a few
// frames per second saturates the link long before battery drain becomes the argument.
//
// The resolution is that PREVIEW QUALITY DOES NOT MATTER. It exists to point the camera, not
// to reconstruct anything. QVGA is roughly 40x less data per frame than QXGA and frames a shot
// just as well.
//
// So the sensor runs at QVGA while previewing and switches up to the capture size only for the
// instant of a capture. That switch is safe, and it does NOT break the exposure lock:
//
//   - The frame buffer is allocated once at init for the configured (largest) frame size, so
//     switching down and back never reallocates - see esp_camera.c.
//   - Exposure is re-applied from the values measured at calibration every time we switch back
//     up. Capture timing is deterministic, so the same aec/gain gives the same exposure on
//     every shot. Preview exposure is left free-running because nobody cares what it is.
// ---------------------------------------------------------------------------
#define CAM_PREVIEW_FRAMESIZE FRAMESIZE_QVGA  // 320x240 - plenty to aim with
#define CAM_PREVIEW_JPEG_QUALITY 14           // lower quality than capture; it is a viewfinder
#define CAM_PREVIEW_INTERVAL_MS 120           // ~8 fps, smooth enough to frame with
#define CAM_PREVIEW_BUF_SIZE (64 * 1024)      // generous for a QVGA JPEG (typically 8-15 KB)

// Frames to discard after switching up to capture resolution, letting the new frame timing
// take effect before we keep an image.
#define CAM_CAPTURE_SETTLE_FRAMES 2

#define CAPTURE_QUEUE_LENGTH 8
#define CAPTURE_TASK_STACK_SIZE 8192

// Below the HTTP server and well below the UART tasks. Writing a photo is never more urgent
// than flying the drone.
#define CAPTURE_TASK_PRIORITY 3

static bool camera_ok;
static bool sd_ok;
static sdmmc_card_t *sd_card;

static QueueHandle_t capture_queue;
static camera_sd_stats_t stats;
static portMUX_TYPE stats_spinlock = portMUX_INITIALIZER_UNLOCKED;

// Exposure measured at boot by camera_sd_calibrate_exposure(). Re-applied every time the
// sensor switches back up to capture resolution, so all captures share one exposure.
static int locked_aec_value;
static int locked_agc_gain;
static bool exposure_locked;

// Frame size the sensor was initialised at - the capture resolution, and the size the frame
// buffers were allocated for. Recorded rather than assumed so the preview logic stays correct
// if a smaller sensor clamped our QXGA request down to its own maximum.
static framesize_t capture_framesize;

// Preview state. `preview_requested` is written by the HTTP handler and read by the capture
// task; `preview_active` is owned entirely by the capture task and reflects what the sensor is
// actually doing. Keeping those separate is what lets the handler return instantly without
// ever touching the sensor.
static volatile bool preview_requested;
static bool preview_active;

// Latest preview JPEG, produced by the capture task and copied out by the HTTP handler.
static uint8_t *preview_buffer;
static size_t preview_length;
static SemaphoreHandle_t preview_mutex;

// Queue tokens. A wake token carries no work; it exists only to break the capture task out of
// its blocking wait when preview is switched on, so it notices the new state immediately.
#define CAPTURE_TOKEN_SHOT 1
#define CAPTURE_TOKEN_WAKE 2

// Measures a correct exposure from the actual scene, then locks it.
//
// Photogrammetry needs the SAME exposure on every shot, otherwise the reconstruction has to
// explain brightness changes as geometry or material changes. But "the same" is not the same
// thing as "a number hard-coded months earlier" - that only works if the lighting happens to
// match whatever was guessed. This runs the sensor's own auto-exposure against the real scene
// for a few frames, reads back what it settled on, and pins the sensor there.
//
// Point the drone at a representative part of the scene while it boots. Everything after this
// call is fixed until the next reboot, which is exactly the property the capture set needs.
//
// Non-fatal by design: if a frame grab fails we simply leave auto-exposure running rather than
// locking to a garbage value, and say so in the log. A camera problem must never stop the
// drone from flying.
static void camera_sd_calibrate_exposure(sensor_t *sensor) {
    // Hand control back to the sensor and let it hunt.
    sensor->set_exposure_ctrl(sensor, 1);
    sensor->set_gain_ctrl(sensor, 1);
    sensor->set_aec2(sensor, 1);
    sensor->set_gainceiling(sensor, GAINCEILING_2X);

    // Exposure bias applied while measuring. Negative biases the result darker, which is what
    // you want here: for reconstruction, slightly dark is recoverable but a blown highlight is
    // gone for good, and a shorter exposure also means less motion blur from a moving drone.
    sensor->set_ae_level(sensor, CAM_AE_LEVEL);

    // Auto-exposure converges over several frames, so throw some away before believing it.
    bool converged = false;
    for (int i = 0; i < CAM_AEC_CALIBRATION_FRAMES; i++) {
        camera_fb_t *frame = esp_camera_fb_get();
        if (frame == NULL) {
            ESP_LOGW(TAG, "Exposure calibration: frame %d/%d failed to grab",
                     i + 1, CAM_AEC_CALIBRATION_FRAMES);
            continue;
        }
        esp_camera_fb_return(frame);
        converged = true;
        vTaskDelay(pdMS_TO_TICKS(CAM_AEC_SETTLE_MS));
    }

    if (!converged) {
        ESP_LOGE(TAG, "Exposure calibration failed - no frames captured. Leaving AEC/AGC on "
                      "(shots will vary between frames, but the camera still works).");
        return;
    }

    // Whatever the sensor chose is now our fixed operating point.
    const int measured_aec = sensor->status.aec_value;
    const int measured_gain = sensor->status.agc_gain;

    sensor->set_exposure_ctrl(sensor, 0);
    sensor->set_aec2(sensor, 0);
    sensor->set_gain_ctrl(sensor, 0);

    // Re-assert the measured values after switching the automatics off: disabling the loop can
    // leave the registers at whatever the last iteration wrote, so write them back explicitly.
    sensor->set_aec_value(sensor, measured_aec);
    sensor->set_agc_gain(sensor, measured_gain);

    // Remember these: every switch back from preview resolution re-applies them, which is what
    // keeps the exposure identical across the whole capture set.
    locked_aec_value = measured_aec;
    locked_agc_gain = measured_gain;
    exposure_locked = true;

    ESP_LOGI(TAG, "Exposure calibrated from scene: aec=%d gain=%d (ae_level bias %d), now LOCKED",
             measured_aec, measured_gain, CAM_AE_LEVEL);
}

// Puts the sensor into viewfinder mode: small frames, quality irrelevant, exposure free to
// wander. Capture-task context only.
static void camera_sd_enter_preview_mode(void) {
    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor == NULL) {
        return;
    }

    sensor->set_framesize(sensor, CAM_PREVIEW_FRAMESIZE);
    sensor->set_quality(sensor, CAM_PREVIEW_JPEG_QUALITY);

    // Let the sensor meter the preview however it likes. The preview is not part of the capture
    // set, so its exposure does not need to match anything - and leaving AEC on means the
    // viewfinder stays usable as you fly between differently lit parts of the room.
    sensor->set_exposure_ctrl(sensor, 1);
    sensor->set_gain_ctrl(sensor, 1);

    preview_active = true;
    ESP_LOGI(TAG, "Preview ON (%ux%u)",
             resolution[CAM_PREVIEW_FRAMESIZE].width,
             resolution[CAM_PREVIEW_FRAMESIZE].height);
}

// Puts the sensor back into capture mode and re-imposes the locked exposure.
// Capture-task context only.
static void camera_sd_enter_capture_mode(void) {
    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor == NULL) {
        return;
    }

    sensor->set_framesize(sensor, capture_framesize);
    sensor->set_quality(sensor, CAM_JPEG_QUALITY);

    // Order matters, exactly as at calibration: kill the automatics first, then write the
    // fixed values, or the next AEC iteration overwrites them.
    sensor->set_exposure_ctrl(sensor, 0);
    sensor->set_aec2(sensor, 0);
    sensor->set_gain_ctrl(sensor, 0);

    if (exposure_locked) {
        sensor->set_aec_value(sensor, locked_aec_value);
        sensor->set_agc_gain(sensor, locked_agc_gain);
    }

    preview_active = false;
}

// Grabs one preview frame and parks it where the HTTP handler can copy it.
// Capture-task context only.
static void camera_sd_refresh_preview(void) {
    camera_fb_t *frame = esp_camera_fb_get();
    if (frame == NULL) {
        return;  // a dropped viewfinder frame is not worth logging every 120 ms
    }

    if (frame->len <= CAM_PREVIEW_BUF_SIZE && preview_buffer != NULL &&
        xSemaphoreTake(preview_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        memcpy(preview_buffer, frame->buf, frame->len);
        preview_length = frame->len;
        xSemaphoreGive(preview_mutex);
    }

    esp_camera_fb_return(frame);
}

// Initialises the camera.
//
// The sensor model is detected at runtime by esp_camera_init(), which probes the chip ID and
// loads the matching driver, so this code is not tied to one sensor. It does however request a
// frame size, and the maximum differs per sensor (driver/sensor.c): the OV2640 tops out at
// UXGA 1600x1200, the OV3660 at QXGA 2048x1536. We ask for QXGA; if a smaller sensor is fitted
// its driver silently clamps to its own maximum, which is why this is safe either way.
static esp_err_t camera_sd_init_camera(void) {
    const camera_config_t camera_config = {
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,

        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_1,      // TIMER_0 / channels 0-3 belong to the motors on the
        .ledc_channel = LEDC_CHANNEL_4,  // flight controller; kept distinct here out of habit

        .pixel_format = PIXFORMAT_JPEG,
        // QXGA 2048x1536 - the OV3660's maximum, and 3.1 MP against UXGA's 1.9 MP. More pixels
        // means more detected features per shot, which is the main thing that limits a Gaussian
        // Splatting reconstruction. The OV3660 driver switches to a 40 MHz SYSCLK / 10 MHz PCLK
        // PLL for QXGA on its own (ov3660.c set_framesize), so 20 MHz XCLK stays correct.
        // Drop to FRAMESIZE_UXGA if capture latency or SD write time turns out to be a problem.
        .frame_size = FRAMESIZE_QXGA,
        .jpeg_quality = CAM_JPEG_QUALITY,
        .fb_count = CAM_FB_COUNT,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_LATEST, // always the freshest frame, not a queued stale one
    };

    esp_err_t error = esp_camera_init(&camera_config);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed: %s", esp_err_to_name(error));
        return error;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor == NULL) {
        ESP_LOGE(TAG, "esp_camera_sensor_get returned NULL");
        return ESP_FAIL;
    }

    // -----------------------------------------------------------------------
    // WHITE BALANCE: LOCKED TO A FIXED PRESET.
    //
    // This is NOT an oversight, and it is the opposite of what an earlier version of this file
    // did. On the OV3660, set_whitebal(sensor, 0) clears bit 0 of ISP_CONTROL_01 (0x5001),
    // which the driver's init table sets to 0x83 to enable the colour matrix, AWB and SDE.
    // Clearing that bit does not FREEZE the white balance - it switches the correction block
    // OFF, so the sensor emits essentially raw Bayer-weighted data. Green then dominates,
    // because a Bayer mosaic has twice as many green photosites as red or blue, and every
    // photo comes out heavily green-cast.
    //
    // What we want instead is the AWB block ENABLED (so gains are applied at all) but running
    // on FIXED gains rather than adapting per frame. set_wb_mode() does exactly that: for any
    // mode 1..4 it writes 0x3406 = 1, which puts the sensor's AWB into manual mode, and then
    // loads constant R/G/B gains into 0x3400 / 0x3402 / 0x3404. The gain values come from the
    // driver itself, so nothing here depends on register numbers I cannot verify.
    //
    // Result: white balance is genuinely locked and identical on every shot - the property the
    // capture set needs - without the green cast that disabling the block produced.
    // -----------------------------------------------------------------------
    sensor->set_whitebal(sensor, 1);        // AWB correction block ON (it applies the gains)
    sensor->set_awb_gain(sensor, 1);        // must be 1, or set_wb_mode is forced back to auto
    sensor->set_wb_mode(sensor, CAM_WB_MODE);

    // -----------------------------------------------------------------------
    // EXPOSURE AND GAIN: MEASURE, THEN LOCK.
    //
    // Constant exposure across a capture set genuinely does matter for reconstruction, so
    // these do get locked - but to a value measured from your actual scene rather than to a
    // constant guessed at compile time. camera_sd_calibrate_exposure() below runs the sensor's
    // own auto-exposure for a few frames, reads back what it converged on, and pins it there.
    // -----------------------------------------------------------------------

    // Leave the correction stages on: they are static per-sensor corrections, not per-frame
    // adaptive ones, so they do not vary shot to shot.
    sensor->set_bpc(sensor, 1);             // black pixel correction
    sensor->set_wpc(sensor, 1);             // white pixel correction
    sensor->set_lenc(sensor, 1);            // lens shading correction

    // No cosmetic processing - it is not reversible and it costs feature detail.
    sensor->set_brightness(sensor, 0);
    sensor->set_contrast(sensor, 0);
    sensor->set_saturation(sensor, 0);
    sensor->set_special_effect(sensor, 0);
    sensor->set_raw_gma(sensor, 1);

    // No flips. If the camera is mounted upside down, fix it here rather than in the
    // reconstruction pipeline - but be consistent across the whole capture set.
    sensor->set_hmirror(sensor, 0);
    sensor->set_vflip(sensor, 0);

    // Report what was actually detected and what the sensor accepted, rather than what we asked
    // for, so a misdetected sensor or a silently clamped setting is visible in the boot log
    // rather than showing up later as an unexplained image problem.
    //
    // This runs LAST: calibration needs the sensor fully configured, and it reports the
    // exposure and gain it settled on itself.
    camera_sd_calibrate_exposure(sensor);

    // Record what the sensor actually settled on rather than assuming FRAMESIZE_QXGA: a smaller
    // sensor clamps our request down to its own maximum, and the preview logic has to switch
    // back to the real capture size, not the one we asked for.
    capture_framesize = sensor->status.framesize;

    camera_sensor_info_t *info = esp_camera_sensor_get_info(&sensor->id);
    ESP_LOGI(TAG, "Camera ready: %s, %ux%u JPEG q%d, WB locked (mode %d), exposure locked",
             info != NULL ? info->name : "unknown sensor",
             resolution[sensor->status.framesize].width,
             resolution[sensor->status.framesize].height,
             CAM_JPEG_QUALITY, CAM_WB_MODE);

    return ESP_OK;
}

// Mounts the SD card over SDMMC in 1-bit mode.
static esp_err_t camera_sd_init_sd(void) {
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    // 1-bit mode: the XIAO Sense only routes DATA0, and it is plenty for the write rate here.
    host.flags = SDMMC_HOST_FLAG_1BIT;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.clk = SD_PIN_CLK;
    slot_config.cmd = SD_PIN_CMD;
    slot_config.d0 = SD_PIN_D0;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        // Deliberately false: silently reformatting the pilot's card because of a transient
        // read error would destroy a whole flight's photos.
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 32 * 1024,
    };

    esp_err_t error = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot_config,
                                              &mount_config, &sd_card);
    if (error != ESP_OK) {
        if (error == ESP_FAIL) {
            ESP_LOGE(TAG, "SD mount failed - is the card formatted FAT32?");
        } else {
            ESP_LOGE(TAG, "SD init failed: %s - is a card inserted?", esp_err_to_name(error));
        }
        return error;
    }

    ESP_LOGI(TAG, "SD card mounted at %s (%lluMB)", SD_MOUNT_POINT,
             ((uint64_t)sd_card->csd.capacity * sd_card->csd.sector_size) / (1024 * 1024));

    return ESP_OK;
}

// Scans the card for existing IMG_#####.JPG files and continues from the highest number,
// so a reboot mid-session does not start overwriting earlier shots.
static void camera_sd_scan_existing_files(void) {
    uint32_t highest = 0;

    DIR *dir = opendir(SD_MOUNT_POINT);
    if (dir == NULL) {
        ESP_LOGW(TAG, "Could not open %s to scan for existing files", SD_MOUNT_POINT);
        stats.next_index = 1;
        return;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        unsigned int index = 0;
        // FAT short names come back upper case; match that.
        if (sscanf(entry->d_name, "IMG_%5u.JPG", &index) == 1) {
            if (index > highest) {
                highest = index;
            }
        }
    }
    closedir(dir);

    stats.next_index = highest + 1;
    ESP_LOGI(TAG, "Existing photos scanned, next file will be IMG_%05lu.JPG",
             (unsigned long)stats.next_index);
}

esp_err_t camera_sd_init(void) {
    capture_queue = xQueueCreate(CAPTURE_QUEUE_LENGTH, sizeof(uint8_t));
    if (capture_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create capture queue");
        return ESP_FAIL;
    }

    memset(&stats, 0, sizeof(stats));
    stats.next_index = 1;

    // --- Camera --------------------------------------------------------------
    // Failure here is logged and swallowed. The drone must still boot and fly.
    if (camera_sd_init_camera() == ESP_OK) {
        camera_ok = true;
    } else {
        ESP_LOGW(TAG, "Camera unavailable - the drone will fly, but cannot take photos");
    }

    // --- SD card -------------------------------------------------------------
    if (camera_sd_init_sd() == ESP_OK) {
        sd_ok = true;
        camera_sd_scan_existing_files();
    } else {
        ESP_LOGW(TAG, "SD card unavailable - the drone will fly, but photos cannot be saved");
    }

    return (camera_ok && sd_ok) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

// Grabs one frame and writes it to the card. Runs only on the capture task.
// Assumes the sensor is already at capture resolution - camera_sd_capture_one() guarantees it.
static void camera_sd_write_one(void) {
    camera_fb_t *frame = esp_camera_fb_get();
    if (frame == NULL) {
        ESP_LOGE(TAG, "esp_camera_fb_get failed");
        portENTER_CRITICAL(&stats_spinlock);
        stats.failed++;
        portEXIT_CRITICAL(&stats_spinlock);
        return;
    }

    portENTER_CRITICAL(&stats_spinlock);
    const uint32_t index = stats.next_index;
    portEXIT_CRITICAL(&stats_spinlock);

    char path[64];
    snprintf(path, sizeof(path), SD_MOUNT_POINT "/IMG_%05lu.JPG", (unsigned long)index);

    FILE *file = fopen(path, "wb");
    if (file == NULL) {
        ESP_LOGE(TAG, "Could not open %s for writing", path);
        esp_camera_fb_return(frame);
        portENTER_CRITICAL(&stats_spinlock);
        stats.failed++;
        portEXIT_CRITICAL(&stats_spinlock);
        return;
    }

    // This is the call that can block for hundreds of milliseconds, which is exactly why it
    // lives on this task and not in the HTTP handler.
    const size_t written = fwrite(frame->buf, 1, frame->len, file);
    fclose(file);

    const size_t frame_len = frame->len;
    esp_camera_fb_return(frame);

    if (written != frame_len) {
        ESP_LOGE(TAG, "Short write to %s: %u of %u bytes (card full?)",
                 path, (unsigned)written, (unsigned)frame_len);
        portENTER_CRITICAL(&stats_spinlock);
        stats.failed++;
        portEXIT_CRITICAL(&stats_spinlock);
        return;
    }

    portENTER_CRITICAL(&stats_spinlock);
    stats.written++;
    stats.next_index++;
    stats.last_size_bytes = (uint32_t)frame_len;
    portEXIT_CRITICAL(&stats_spinlock);

    ESP_LOGI(TAG, "Saved %s (%u bytes)", path, (unsigned)frame_len);
}

// Takes one photo at capture resolution, whatever mode the sensor happens to be in.
//
// Wrapping camera_sd_write_one() rather than folding the resolution handling into it is
// deliberate: the write path has several early returns for grab and card failures, and doing
// the switch-back here means it cannot be skipped by any of them. Leaving the sensor stuck at
// capture resolution would silently kill the viewfinder after the first failed shot.
static void camera_sd_capture_one(void) {
    const bool was_previewing = preview_active;

    if (was_previewing) {
        // The sensor is at QVGA with its own auto-exposure. Switch up and re-impose the locked
        // exposure so this shot matches every other shot in the set.
        camera_sd_enter_capture_mode();

        // Frames already in flight were exposed under the old timing. Discard them, otherwise
        // the first shot after each preview toggle comes out wrongly exposed.
        for (int i = 0; i < CAM_CAPTURE_SETTLE_FRAMES; i++) {
            camera_fb_t *stale = esp_camera_fb_get();
            if (stale != NULL) {
                esp_camera_fb_return(stale);
            }
        }
    }

    camera_sd_write_one();

    if (was_previewing) {
        camera_sd_enter_preview_mode();
    }
}

static void camera_sd_task(void *args) {
    ESP_LOGI(TAG, "Capture task started on core %d", xPortGetCoreID());

    for (;;) {
        // With preview off this blocks indefinitely and costs no CPU at all, which is the
        // normal in-flight case. With preview on it wakes every frame interval to refresh the
        // viewfinder. Either way a capture request is serviced the moment it arrives.
        const TickType_t wait = preview_active ? pdMS_TO_TICKS(CAM_PREVIEW_INTERVAL_MS)
                                               : portMAX_DELAY;

        uint8_t token = 0;
        if (xQueueReceive(capture_queue, &token, wait) == pdTRUE) {
            if (token == CAPTURE_TOKEN_SHOT) {
                camera_sd_capture_one();
            }
            // A wake token carries no work - it exists purely to break the blocking wait above
            // so the preview state change below is noticed straight away.
        }

        // Apply any pending preview toggle. All sensor access lives on this task, so the HTTP
        // handler only ever sets a flag and returns; there is no lock on the sensor and no way
        // for a request to stall on it.
        if (preview_requested != preview_active) {
            if (preview_requested) {
                camera_sd_enter_preview_mode();
            } else {
                camera_sd_enter_capture_mode();
                ESP_LOGI(TAG, "Preview OFF, exposure re-locked (aec=%d gain=%d)",
                         locked_aec_value, locked_agc_gain);
            }
        }

        if (preview_active) {
            camera_sd_refresh_preview();
        }
    }
}

// Turns the viewfinder on or off. Returns immediately - the switch is applied by the capture
// task, which owns the sensor.
esp_err_t camera_sd_set_preview(bool enable) {
    if (!camera_ok) {
        return ESP_ERR_INVALID_STATE;
    }

    preview_requested = enable;

    // Nudge the capture task so it notices without waiting out a blocking receive.
    if (capture_queue != NULL) {
        const uint8_t token = CAPTURE_TOKEN_WAKE;
        xQueueSend(capture_queue, &token, 0);
    }

    return ESP_OK;
}

bool camera_sd_preview_enabled(void) {
    return preview_requested;
}

size_t camera_sd_copy_preview(uint8_t *dst, size_t dst_capacity) {
    if (dst == NULL || preview_buffer == NULL || preview_mutex == NULL) {
        return 0;
    }

    size_t copied = 0;
    if (xSemaphoreTake(preview_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        if (preview_length > 0 && preview_length <= dst_capacity) {
            memcpy(dst, preview_buffer, preview_length);
            copied = preview_length;
        }
        xSemaphoreGive(preview_mutex);
    }

    return copied;
}

esp_err_t camera_sd_start_task(BaseType_t core_id) {
    // Preview buffer lives in PSRAM - too big to spare from internal SRAM, and nothing touches
    // it from an ISR. If it cannot be allocated the viewfinder is simply unavailable; that must
    // not stop the capture task starting, because captures still work without it.
    preview_buffer = heap_caps_malloc(CAM_PREVIEW_BUF_SIZE, MALLOC_CAP_SPIRAM);
    preview_mutex = xSemaphoreCreateMutex();
    if (preview_buffer == NULL || preview_mutex == NULL) {
        ESP_LOGW(TAG, "Preview buffer unavailable - viewfinder disabled, captures unaffected");
    }

    BaseType_t result = xTaskCreatePinnedToCore(
        camera_sd_task,
        "CAMERA_SD",
        CAPTURE_TASK_STACK_SIZE,
        NULL,
        CAPTURE_TASK_PRIORITY,
        NULL,
        core_id);

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create capture task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t camera_sd_request_capture(void) {
    if (!camera_ok || !sd_ok || capture_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    const uint8_t token = CAPTURE_TOKEN_SHOT;

    // Zero timeout. This is called from the HTTP handler; it queues and returns, and never
    // waits for the grab or the write.
    if (xQueueSend(capture_queue, &token, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Capture queue full, dropping request");
        portENTER_CRITICAL(&stats_spinlock);
        stats.failed++;
        portEXIT_CRITICAL(&stats_spinlock);
        return ESP_ERR_NO_MEM;
    }

    portENTER_CRITICAL(&stats_spinlock);
    stats.requested++;
    portEXIT_CRITICAL(&stats_spinlock);

    return ESP_OK;
}

void camera_sd_get_stats(camera_sd_stats_t *out) {
    if (out == NULL) {
        return;
    }

    portENTER_CRITICAL(&stats_spinlock);
    *out = stats;
    portEXIT_CRITICAL(&stats_spinlock);
}

bool camera_sd_camera_ok(void) {
    return camera_ok;
}

bool camera_sd_sd_ok(void) {
    return sd_ok;
}
