#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

// Thin wrapper around ST's VL53L1X Ultra Lite Driver.
//
// The sensor shares the I2C bus that imu_driver already created on I2C_NUM_0; this component
// claims that bus with i2c_master_get_bus_handle() rather than creating a second one, so
// imu_driver is untouched and imu_setup() must run first.
//
// ST's ULD sources live in st_uld/ (STSW-IMG009 v3.5.5, unmodified, BSD SLA0103). The
// CMakeLists detects them and defines VL53L1X_ULD_PRESENT. If they are ever removed the
// component still builds, but vl53l1x_init() returns ESP_ERR_NOT_SUPPORTED, nav_estimator
// reports altitude as invalid, and flight_control disengages the altitude and position loops
// so the drone stays flyable in angle mode. See st_uld/README.md.

// One ranging result.
typedef struct {
    uint16_t distance_mm;   // raw line-of-sight range, NOT tilt compensated
    uint8_t range_status;   // 0 = good; anything else means do not trust distance_mm
    bool valid;             // convenience: range_status == 0 and the read succeeded
} vl53l1x_result_t;

// Initialises the sensor: claims the shared I2C bus, boots the device, loads ST's default
// configuration and starts continuous ranging.
// Safe to call when the hardware is absent - it logs and returns an error rather than
// aborting, so a missing ToF sensor never stops the drone from booting.
// @return ESP_OK on success, ESP_ERR_NOT_SUPPORTED if the ST ULD sources are not present,
//         or an error code if the sensor did not respond.
esp_err_t vl53l1x_init(void);

// Reads the most recent measurement and clears the interrupt so the next one can start.
// Non-blocking: if no new measurement is ready it returns ESP_ERR_NOT_FINISHED and leaves
// `out` untouched.
// @param out Destination for the result, must not be NULL.
// @return ESP_OK on a fresh reading, ESP_ERR_NOT_FINISHED if data is not ready,
//         ESP_ERR_INVALID_STATE if the driver never initialised, or an I2C error.
esp_err_t vl53l1x_read(vl53l1x_result_t *out);

// True if init succeeded and the sensor is ranging.
bool vl53l1x_is_available(void);

// Hands the shared I2C device handle to the platform layer. Called by vl53l1x_init().
// @param handle The i2c_master device handle for the VL53L1X.
void vl53l1_platform_set_handle(i2c_master_dev_handle_t handle);
