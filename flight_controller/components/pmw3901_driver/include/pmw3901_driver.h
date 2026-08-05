#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// PixArt PMW3901 optical flow sensor, over SPI.
//
// >>> THIS DRIVER IS NOT COMPLETE AND WILL NOT INITIALISE. <<<
// The ~80-register power-up sequence PixArt require is deliberately NOT written here - see
// pmw3901_write_init_sequence() in pmw3901_driver.c. Those values are undocumented magic from
// PixArt's application note and guessing them would at best not work and at worst damage the
// part. Transcribe them by hand from Bitcraze's implementation:
//
//     https://github.com/bitcraze/Bitcraze_PMW3901
//     (see initRegisters() in src/Bitcraze_PMW3901.cpp)
//
// Until that is done, pmw3901_init() returns ESP_ERR_NOT_SUPPORTED. nav_estimator then
// reports velocity_valid == false, flight_control disengages the velocity outer loop, and the
// drone still flies normally in angle mode and altitude hold.

// One optical flow reading.
typedef struct {
    int16_t delta_x;  // accumulated motion since the last read, sensor counts (not pixels, not metres)
    int16_t delta_y;  // sensor counts
    uint8_t squal;    // surface quality, higher is better. Low values mean a featureless floor.
    bool motion;      // the sensor's own "motion occurred" flag
    bool valid;       // the read succeeded and squal cleared the usable threshold
} pmw3901_motion_t;

// Configures the SPI bus and device, verifies the product ID, and runs the power-up sequence.
// Safe to call with no sensor attached - it logs and returns an error rather than aborting.
// @return ESP_OK on success, ESP_ERR_NOT_SUPPORTED while the init register sequence is still
//         a stub, or an error code if the SPI bus or the product ID check failed.
esp_err_t pmw3901_init(void);

// Reads accumulated motion since the previous call. The sensor accumulates internally, so the
// deltas are relative to the last read, not to any fixed origin.
// @param out Destination, must not be NULL.
// @return ESP_OK on success, ESP_ERR_INVALID_STATE if the driver never initialised,
//         or an error code on an SPI failure.
esp_err_t pmw3901_read_motion(pmw3901_motion_t *out);

// True if init succeeded and the sensor is usable.
bool pmw3901_is_available(void);
