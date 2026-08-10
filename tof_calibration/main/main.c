// main.c - VL53L1X offset calibration bench tool.
//
// WHAT THIS PROGRAM DOES
//   Produces ONE number: the value that belongs in VL53L1X_OFFSET_MM in
//   ../flight_controller/components/vl53l1x_driver/vl53l1x_driver.c, which is currently 0
//   (TODO(bench) there). It runs on a spare ESP32-S3 with nothing wired but the sensor, because
//   holding a grey card at an exact distance from a drone with four live motor outputs is not a
//   thing anyone should do.
//
// HOW IT DOES IT
//   Four phases, all over the serial monitor:
//     1. Creates the I2C bus the driver expects. On the drone, imu_setup() creates I2C_NUM_0 and
//        vl53l1x_init() borrows it; there is no IMU here, so this file creates it instead. That
//        is the ONLY thing this program does differently from the drone.
//     2. Calls the REAL vl53l1x_init() from the real component - short mode, 20 ms budget, 25 ms
//        inter-measurement, offset 0. This matters: the offset is only valid for the ranging
//        configuration it was measured under, so calibrating with a hand-rolled init would give
//        a number that is subtly wrong in flight.
//     3. Live positioning aid - prints the raw distance twice a second while you slide the target
//        to exactly CAL_TARGET_DISTANCE_MM, then a stability check before committing.
//     4. Runs ST's VL53L1X_CalibrateOffset(), prints the result, then restarts ranging and prints
//        live CALIBRATED readings forever so you can verify against a ruler.
//
// WHY THE VERIFICATION PHASE EXISTS
//   CalibrateOffset() writes the offset it found into the device, so the phase-4 readings already
//   include it. Checking two known distances against a ruler is a real end-to-end test of the
//   number before you paste it into the flight firmware.

#include <stdio.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"

#include "vl53l1x_driver.h"

#ifdef VL53L1X_ULD_PRESENT
#include "VL53L1X_api.h"
#include "VL53L1X_calibration.h"
#endif

static const char *TAG = "TOF_CAL";

// ---------------------------------------------------------------------------
// Wiring. These are the CALIBRATION RIG's pins, not the drone's - change them freely to match
// however you have the spare board wired. The drone uses SDA 12 / SCL 11 (imu_driver.c creates
// that bus); matching it here is convenient but not required, because nothing about the offset
// depends on which pins the bus runs on.
//
// XSHUT is the exception: vl53l1x_driver.c hardcodes GPIO 17 and pulses it low->high at init.
// Either wire XSHUT to GPIO 17 on this board too, or tie XSHUT to 3V3 (in which case the pulse
// on an unconnected GPIO 17 is a harmless no-op). Do NOT leave XSHUT floating - the part may
// stay in reset.
// ---------------------------------------------------------------------------
#define CAL_I2C_SDA_GPIO 5
#define CAL_I2C_SCL_GPIO 6

// The VL53L1X's 7-bit address. ST's ULD takes this as its opaque `dev` argument; it must match
// the address vl53l1x_init() registered the device at.
#define CAL_I2C_ADDRESS 0x29

// ---------------------------------------------------------------------------
// The target distance, in mm.
//
// THE ONLY HARD REQUIREMENT: this number must equal the distance you physically place the target
// at. ST computes offset = CAL_TARGET_DISTANCE_MM - (average measured), so a ruler that disagrees
// with this #define puts the disagreement straight into the offset.
//
// 140 mm matches the procedure documented in the component README. ST's own
// VL53L1X_calibration.h suggests 100 mm. Either works provided the ruler matches this value.
// Target must be 17% grey reflectance (ST's spec) - see README.md for what to use.
// ---------------------------------------------------------------------------
#define CAL_TARGET_DISTANCE_MM 140

// How far off the target the pre-check average may be before we warn. A real cover-glass offset
// is typically within a couple of centimetres; tens of mm usually means the ruler is wrong.
#define CAL_SANITY_WARN_MM 40

// Samples averaged for the pre-check. ST's CalibrateOffset() internally averages 50.
#define CAL_PRECHECK_SAMPLES 30

// Seconds of live readout before calibration starts, to position the target.
#define CAL_POSITIONING_SECONDS 20

#ifdef VL53L1X_ULD_PRESENT

// Averages `wanted` fresh measurements. Only measurements with range_status == 0 feed the mean;
// the count of those is reported separately so the caller can judge whether the target is being
// seen reliably.
//
// @return ESP_OK, ESP_ERR_TIMEOUT if the sensor stopped producing measurements, or
//         ESP_ERR_INVALID_RESPONSE if measurements arrived but none of them were valid.
static esp_err_t sample_mean(int wanted, float *mean_out, int *valid_out, uint8_t *first_bad_status)
{
    uint32_t sum = 0;
    int got = 0;
    int valid = 0;
    *first_bad_status = 0;

    // Each measurement takes ~25 ms and we poll every 2 ms, so ~13 polls per sample. The margin
    // is generous because a bounded loop that gives up beats a bench tool that hangs.
    const int max_polls = wanted * 60 + 500;

    for (int poll = 0; poll < max_polls && got < wanted; poll++) {
        vl53l1x_result_t result;
        esp_err_t error = vl53l1x_read(&result);

        if (error == ESP_ERR_NOT_FINISHED) {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        if (error != ESP_OK) {
            return error;
        }

        got++;
        if (result.valid) {
            sum += result.distance_mm;
            valid++;
        } else if (*first_bad_status == 0) {
            *first_bad_status = result.range_status;
        }
    }

    if (got < wanted) {
        return ESP_ERR_TIMEOUT;
    }
    if (valid == 0) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    *mean_out = (float)sum / (float)valid;
    *valid_out = valid;
    return ESP_OK;
}

// Phase 1: create the bus vl53l1x_init() expects to find. This is the drone's imu_setup()
// standing in - the driver calls i2c_master_get_bus_handle(I2C_NUM_0, ...) and will fail with
// "did imu_setup() run first?" if nothing has created it.
static esp_err_t calibration_bus_setup(void)
{
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CAL_I2C_SDA_GPIO,
        .scl_io_num = CAL_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        // Most VL53L1X breakouts carry their own 2.2k-10k pull-ups. The internal ones are weak
        // (~45k) and are here only so a bare module without pull-ups still enumerates at 400 kHz.
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle = NULL;
    return i2c_new_master_bus(&bus_config, &bus_handle);
}

// Phase 3: print the raw distance while the user positions the target.
static void positioning_aid(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== POSITION THE TARGET =================================");
    ESP_LOGI(TAG, "Place the 17%% grey target at EXACTLY %d mm from the sensor face.",
             CAL_TARGET_DISTANCE_MM);
    ESP_LOGI(TAG, "Readings below are UNCALIBRATED - they are not supposed to read %d yet.",
             CAL_TARGET_DISTANCE_MM);
    ESP_LOGI(TAG, "Measure with the ruler, not with this number. Starting in %d s.",
             CAL_POSITIONING_SECONDS);
    ESP_LOGI(TAG, "=========================================================");

    for (int tick = 0; tick < CAL_POSITIONING_SECONDS * 2; tick++) {
        float mean = 0.0f;
        int valid = 0;
        uint8_t bad_status = 0;

        if (sample_mean(3, &mean, &valid, &bad_status) == ESP_OK) {
            ESP_LOGI(TAG, "[%2d s left] raw %.1f mm", CAL_POSITIONING_SECONDS - tick / 2, mean);
        } else {
            ESP_LOGW(TAG, "[%2d s left] no valid measurement (status %u) - target in view?",
                     CAL_POSITIONING_SECONDS - tick / 2, bad_status);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Phase 4: ranging is stopped by CalibrateOffset(), so restart it and stream calibrated readings.
static void verification_loop(void)
{
    // CalibrateOffset() ends with StopRanging, which can leave an interrupt latched from its last
    // measurement. Averaging 10 samples would wash a stale first reading out anyway, but clearing
    // it is free.
    VL53L1X_ClearInterrupt(CAL_I2C_ADDRESS);

    if (VL53L1X_StartRanging(CAL_I2C_ADDRESS) != 0) {
        ESP_LOGE(TAG, "Could not restart ranging for verification - the offset above is still good.");
        return;
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== VERIFICATION ========================================");
    ESP_LOGI(TAG, "These readings now INCLUDE the offset that was just programmed.");
    ESP_LOGI(TAG, "Check them against a ruler at two distances, e.g. %d mm and 500 mm.",
             CAL_TARGET_DISTANCE_MM);
    ESP_LOGI(TAG, "Short mode tops out around 1.3 m. Ctrl-] to quit the monitor.");
    ESP_LOGI(TAG, "=========================================================");

    while (true) {
        float mean = 0.0f;
        int valid = 0;
        uint8_t bad_status = 0;

        if (sample_mean(10, &mean, &valid, &bad_status) == ESP_OK) {
            ESP_LOGI(TAG, "calibrated %.1f mm  (%d/10 valid)", mean, valid);
        } else {
            ESP_LOGW(TAG, "no valid measurement (status %u)", bad_status);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

#endif  // VL53L1X_ULD_PRESENT

void app_main(void)
{
#ifndef VL53L1X_ULD_PRESENT
    ESP_LOGE(TAG, "ST's VL53L1X ULD sources are not present in");
    ESP_LOGE(TAG, "flight_controller/components/vl53l1x_driver/st_uld/ - nothing to calibrate.");
    ESP_LOGE(TAG, "See that directory's README.md.");
    return;
#else
    ESP_LOGI(TAG, "VL53L1X offset calibration bench tool");
    ESP_LOGI(TAG, "I2C SDA=%d SCL=%d, XSHUT=GPIO17 (hardcoded in vl53l1x_driver.c)",
             CAL_I2C_SDA_GPIO, CAL_I2C_SCL_GPIO);

    esp_err_t error = calibration_bus_setup();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Could not create the I2C bus: %s", esp_err_to_name(error));
        return;
    }

    // The real driver, with the real flight configuration. Do not substitute a local init here.
    error = vl53l1x_init();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "vl53l1x_init() failed: %s", esp_err_to_name(error));
        ESP_LOGE(TAG, "Check 3V3/GND, SDA/SCL, and that XSHUT is high (GPIO 17 or tied to 3V3).");
        return;
    }

    positioning_aid();

    // Pre-check: a wildly wrong average here almost always means the ruler disagrees with
    // CAL_TARGET_DISTANCE_MM, which would be baked silently into the offset.
    float precheck_mean = 0.0f;
    int precheck_valid = 0;
    uint8_t precheck_status = 0;

    error = sample_mean(CAL_PRECHECK_SAMPLES, &precheck_mean, &precheck_valid, &precheck_status);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Pre-check failed (%s, status %u) - is the target in view?",
                 esp_err_to_name(error), precheck_status);
        return;
    }

    const float precheck_error = precheck_mean - (float)CAL_TARGET_DISTANCE_MM;
    ESP_LOGI(TAG, "Pre-check: %.1f mm average over %d/%d valid samples (%.1f mm from target)",
             precheck_mean, precheck_valid, CAL_PRECHECK_SAMPLES, precheck_error);

    if (precheck_error > CAL_SANITY_WARN_MM || precheck_error < -CAL_SANITY_WARN_MM) {
        ESP_LOGW(TAG, "That is more than %d mm off. A genuine cover-glass offset is usually",
                 CAL_SANITY_WARN_MM);
        ESP_LOGW(TAG, "much smaller - check the ruler and CAL_TARGET_DISTANCE_MM agree.");
        ESP_LOGW(TAG, "Continuing anyway; judge the result below for yourself.");
    }
    if (precheck_valid < CAL_PRECHECK_SAMPLES) {
        ESP_LOGW(TAG, "%d of %d samples were invalid (first bad status %u) - a flaky target",
                 CAL_PRECHECK_SAMPLES - precheck_valid, CAL_PRECHECK_SAMPLES, precheck_status);
        ESP_LOGW(TAG, "makes for a noisy offset. Check lighting and target reflectance.");
    }

    // ST averages 50 measurements internally, then writes the offset it found into the device.
    ESP_LOGI(TAG, "Calibrating (ST averages 50 measurements, ~2 s)...");

    int16_t offset_mm = 0;
    const int8_t status = VL53L1X_CalibrateOffset(CAL_I2C_ADDRESS, CAL_TARGET_DISTANCE_MM, &offset_mm);

    // Nonzero includes ST's 1000-poll timeout path, which returns EARLY without StopRanging and
    // leaves `offset_mm` untouched. Printing it in that case would be printing a garbage number
    // into the flight firmware.
    if (status != 0) {
        ESP_LOGE(TAG, "VL53L1X_CalibrateOffset() failed with status %d - no usable offset.", status);
        ESP_LOGE(TAG, "Most likely the sensor stopped returning measurements mid-run.");
        return;
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "#########################################################");
    ESP_LOGI(TAG, "##  OFFSET = %d mm", offset_mm);
    ESP_LOGI(TAG, "##");
    ESP_LOGI(TAG, "##  Put this in vl53l1x_driver.c, replacing the 0:");
    ESP_LOGI(TAG, "##      #define VL53L1X_OFFSET_MM %d", offset_mm);
    ESP_LOGI(TAG, "#########################################################");

    // Free cross-check: the pre-check ran at offset 0 and ST computes offset = target - average,
    // so the number above should be about -(pre-check error). A real disagreement means the
    // target moved between the two phases, which invalidates the result.
    const float expected_offset = -precheck_error;
    const float disagreement = (float)offset_mm - expected_offset;
    if (disagreement > 5.0f || disagreement < -5.0f) {
        ESP_LOGW(TAG, "Cross-check: the pre-check predicted about %.1f mm, ST returned %d mm.",
                 expected_offset, offset_mm);
        ESP_LOGW(TAG, "A gap that size usually means the target moved mid-run. Re-run before");
        ESP_LOGW(TAG, "trusting this number.");
    } else {
        ESP_LOGI(TAG, "Cross-check OK: pre-check predicted about %.1f mm.", expected_offset);
    }

    verification_loop();
#endif
}
