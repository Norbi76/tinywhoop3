#include "vl53l1x_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

// VL53L1X_ULD_PRESENT is defined by CMakeLists when the four ST files are found in st_uld/.
#ifdef VL53L1X_ULD_PRESENT
#include "VL53L1X_api.h"
#include "VL53L1X_calibration.h"
#endif

static const char *TAG = "VL53L1X";

// Default 7-bit I2C address of the VL53L1X. Does not clash with the MPU6500 at 0x68.
#define VL53L1X_I2C_ADDRESS 0x29
#define VL53L1X_I2C_FREQ_HZ 400000

// ---------------------------------------------------------------------------
// TODO(pins): CONFIRM AGAINST YOUR WIRING.
// XSHUT is the sensor's active-low shutdown pin. It must be driven high to bring the part out
// of reset. If you have hard-wired XSHUT to 3V3 instead of a GPIO, set this to -1 and the
// driver will skip the reset pulse.
// GPIO 11 and 12 are the I2C bus (imu_driver.c). GPIO 4, 5, 15, 16 are the motors.
// ---------------------------------------------------------------------------
#define VL53L1X_XSHUT_GPIO 17

// ---------------------------------------------------------------------------
// TODO(bench): VL53L1X_OFFSET_MM - MEASURE THIS.
// The ToF reports a systematic offset that depends on the cover glass in front of it.
// ST's procedure: place a 17% grey target at exactly 140 mm, run VL53L1X_CalibrateOffset()
// once, and put the value it returns here. Leaving it at 0 typically costs a couple of
// centimetres of absolute altitude accuracy, which matters for altitude hold.
// ---------------------------------------------------------------------------
#define VL53L1X_OFFSET_MM 0

// Short distance mode: ~1.3 m ceiling but far better immunity to ambient IR. Correct choice
// for an indoor drone flying low over a floor. Long mode would reach 4 m but is unusable
// under bright room lighting.
#define VL53L1X_DISTANCE_MODE_SHORT 1

// 20 ms is the minimum timing budget for short mode; 25 ms between measurements gives 40 Hz,
// comfortably faster than the 33 Hz at which sensor_task actually reads it.
#define VL53L1X_TIMING_BUDGET_MS 20
#define VL53L1X_INTER_MEASUREMENT_MS 25

static bool sensor_available;
static i2c_master_dev_handle_t device_handle;

// Pulses XSHUT to bring the sensor up in a known state.
// Harmless to skip - some boards tie XSHUT high permanently.
static void vl53l1x_hardware_reset(void) {
    if (VL53L1X_XSHUT_GPIO < 0) {
        return;
    }

    const gpio_config_t xshut_config = {
        .pin_bit_mask = (1ULL << VL53L1X_XSHUT_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&xshut_config);

    gpio_set_level(VL53L1X_XSHUT_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(VL53L1X_XSHUT_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));  // datasheet allows 1.2 ms to boot; 10 ms is generous
}

esp_err_t vl53l1x_init(void) {
#ifndef VL53L1X_ULD_PRESENT
    ESP_LOGE(TAG, "ST VL53L1X ULD sources are missing - the ToF sensor is disabled.");
    ESP_LOGE(TAG, "Drop VL53L1X_api.c/.h and VL53L1X_calibration.c/.h into");
    ESP_LOGE(TAG, "components/vl53l1x_driver/st_uld/ and rebuild. See st_uld/README.md.");
    sensor_available = false;
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (sensor_available) {
        return ESP_OK;
    }

    // Reuse the bus imu_driver already created rather than configuring a second master on the
    // same pins. imu_setup() must therefore have run before this point.
    i2c_master_bus_handle_t bus_handle = NULL;
    esp_err_t error = i2c_master_get_bus_handle(I2C_NUM_0, &bus_handle);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Shared I2C bus not available (did imu_setup() run first?): %s",
                 esp_err_to_name(error));
        return error;
    }

    vl53l1x_hardware_reset();

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53L1X_I2C_ADDRESS,
        .scl_speed_hz = VL53L1X_I2C_FREQ_HZ,
    };

    error = i2c_master_bus_add_device(bus_handle, &device_config, &device_handle);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add VL53L1X at 0x%02x: %s", VL53L1X_I2C_ADDRESS, esp_err_to_name(error));
        return error;
    }

    vl53l1_platform_set_handle(device_handle);

    // ---------------------------------------------------------------------
    // From here down we are calling into ST's ULD (STSW-IMG009 v3.5.5, see st_uld/).
    // Every call below is checked against VL53L1X_api.h and links cleanly.
    // ---------------------------------------------------------------------
    uint8_t boot_state = 0;
    for (int attempt = 0; attempt < 100; attempt++) {
        if (VL53L1X_BootState(VL53L1X_I2C_ADDRESS, &boot_state) == 0 && boot_state != 0) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (boot_state == 0) {
        ESP_LOGE(TAG, "VL53L1X never finished booting - check wiring and XSHUT");
        return ESP_ERR_TIMEOUT;
    }

    if (VL53L1X_SensorInit(VL53L1X_I2C_ADDRESS) != 0) {
        ESP_LOGE(TAG, "VL53L1X_SensorInit failed");
        return ESP_FAIL;
    }

    VL53L1X_SetDistanceMode(VL53L1X_I2C_ADDRESS, VL53L1X_DISTANCE_MODE_SHORT);
    VL53L1X_SetTimingBudgetInMs(VL53L1X_I2C_ADDRESS, VL53L1X_TIMING_BUDGET_MS);
    VL53L1X_SetInterMeasurementInMs(VL53L1X_I2C_ADDRESS, VL53L1X_INTER_MEASUREMENT_MS);
    VL53L1X_SetOffset(VL53L1X_I2C_ADDRESS, VL53L1X_OFFSET_MM);

    if (VL53L1X_StartRanging(VL53L1X_I2C_ADDRESS) != 0) {
        ESP_LOGE(TAG, "VL53L1X_StartRanging failed");
        return ESP_FAIL;
    }

    sensor_available = true;
    ESP_LOGI(TAG, "VL53L1X ranging (short mode, %d ms budget, offset %d mm)",
             VL53L1X_TIMING_BUDGET_MS, VL53L1X_OFFSET_MM);

    return ESP_OK;
#endif
}

esp_err_t vl53l1x_read(vl53l1x_result_t *out) {
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sensor_available) {
        return ESP_ERR_INVALID_STATE;
    }

#ifndef VL53L1X_ULD_PRESENT
    return ESP_ERR_NOT_SUPPORTED;
#else
    uint8_t data_ready = 0;
    if (VL53L1X_CheckForDataReady(VL53L1X_I2C_ADDRESS, &data_ready) != 0) {
        return ESP_FAIL;
    }
    if (!data_ready) {
        return ESP_ERR_NOT_FINISHED;
    }

    uint16_t distance_mm = 0;
    uint8_t range_status = 0;

    if (VL53L1X_GetDistance(VL53L1X_I2C_ADDRESS, &distance_mm) != 0 ||
        VL53L1X_GetRangeStatus(VL53L1X_I2C_ADDRESS, &range_status) != 0) {
        return ESP_FAIL;
    }

    // Must clear the interrupt or the sensor never produces another measurement.
    VL53L1X_ClearInterrupt(VL53L1X_I2C_ADDRESS);

    out->distance_mm = distance_mm;
    out->range_status = range_status;
    out->valid = (range_status == 0);

    return ESP_OK;
#endif
}

bool vl53l1x_is_available(void) {
    return sensor_available;
}
