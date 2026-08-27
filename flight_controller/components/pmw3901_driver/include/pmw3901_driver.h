#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>   // size_t, for the frame readout API
#include "esp_err.h"

// pmw3901_driver.h - downward-facing optical flow sensor, the horizontal-velocity sensor.
//
// WHAT THIS COMPONENT IS FOR
//   Watches the floor and reports how far the image shifted since the last read. nav_estimator
//   scales that by altitude (from the ToF) to get body-frame velocity, and flight_control's outer
//   loop drives that velocity to zero for position hold. This layer reports raw sensor COUNTS and
//   a surface-quality number - no scaling to physical units, no altitude, no fusion.
//
// HOW IT DOES IT
//   SPI mode 3 at 2 MHz with MANUALLY driven chip select (a register read needs CS held low
//   across address -> delay -> data, which hardware CS would break by deasserting in between),
//   plus busy-wait gaps between transactions that PixArt require.
//
// READ THE WARNING BELOW BEFORE FLYING THIS.
//
// PixArt PMW3901 optical flow sensor, over SPI.
//
// The ~73-register power-up sequence PixArt require now lives in pmw3901_init_registers.h,
// transcribed from Bitcraze's MIT-licensed driver. So pmw3901_init() can now succeed - it no
// longer returns ESP_ERR_NOT_SUPPORTED unconditionally.
//
// >>> THAT REMOVED A SAFETY PROPERTY. <<<
// While init always failed, velocity_valid was permanently false and flight_control could never
// engage the velocity outer loop. It can now. The scale factor that turns these counts into a
// velocity - FLOW_COUNTS_PER_RAD in nav_estimator.c - is still an unmeasured guess, and the
// gyro-compensation axis signs there are unverified. A sign error is positive feedback: the
// drone accelerates away from the hold point instead of settling on it.
//
// Measure the constant and check the signs with ../../../flow_calibration/ before flying.
// This driver has never been run against real hardware.

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

// --- Raw frame readout: BENCH DIAGNOSTIC ONLY -------------------------------
// Nothing in the flight path calls these, and nothing in the flight path should.
//
// The sensor can stream its raw 35x35 image out over SPI, which answers one question the motion
// registers cannot: what is ACTUALLY in the field of view. Anything at a different depth from
// the target plane - a table edge, the clamp holding the sensor - corrupts the flow correlation
// while frequently leaving squal looking perfectly healthy.
//
// >>> ENTERING FRAME MODE PERMANENTLY STOPS MOTION REPORTING. <<<
// Frame mode partially overwrites registers the power-up table set, so there is no clean undo.
// **Calling pmw3901_init() again will NOT recover it** - that function is idempotent and returns
// ESP_OK immediately once the driver is up, leaving you with a sensor that reports nothing.
// Power-cycle the sensor so init genuinely runs again.
// ../../../flow_calibration/ therefore selects frame view at COMPILE time (CAL_FRAME_VIEW) so
// the two modes can never be confused at runtime.

#define PMW3901_FRAME_WIDTH  35
#define PMW3901_FRAME_HEIGHT 35
#define PMW3901_FRAME_PIXELS (PMW3901_FRAME_WIDTH * PMW3901_FRAME_HEIGHT)

// Switches the sensor into image readout mode. Call once, before pmw3901_capture_frame().
// @return ESP_OK, ESP_ERR_INVALID_STATE if the driver never initialised, ESP_ERR_TIMEOUT if the
//         sensor never signalled ready, or an SPI error.
esp_err_t pmw3901_frame_mode_enter(void);

// Reads one full frame. Each byte is a 6-bit greyscale sample (0..63), row-major.
// Slow - around 1225 pixels of two-byte SPI reads, so roughly half a second per frame.
// @param pixels Destination, at least PMW3901_FRAME_PIXELS bytes.
// @param pixel_count Size of that buffer.
// @return ESP_OK, ESP_ERR_INVALID_ARG if the buffer is NULL or too small,
//         ESP_ERR_INVALID_STATE if the driver never initialised, ESP_ERR_TIMEOUT if the pixel
//         stream stalled, or an SPI error.
esp_err_t pmw3901_capture_frame(uint8_t *pixels, size_t pixel_count);
