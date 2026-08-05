#include "vl53l1_platform.h"
#include "vl53l1x_driver.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "VL53L1X_PLAT";

// The largest single transfer the ULD asks for. ST's api uses ReadMulti for the 17-byte
// results block and WriteMulti for short config bursts; 64 bytes leaves plenty of headroom.
#define VL53L1_MAX_TRANSFER 64

// I2C transaction timeout. The VL53L1X clock-stretches during ranging, so this cannot be
// zero, but it must stay well under the sensor_task period so a dead sensor cannot stall
// the 100 Hz loop.
#define VL53L1_I2C_TIMEOUT_MS 20

// Set by vl53l1x_driver.c once the shared bus handle has been claimed.
static i2c_master_dev_handle_t sensor_handle;

void vl53l1_platform_set_handle(i2c_master_dev_handle_t handle) {
    sensor_handle = handle;
}

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    (void)dev;  // single sensor on this bus; the handle is held in this module

    if (sensor_handle == NULL || pdata == NULL) {
        return -1;
    }
    if (count > (VL53L1_MAX_TRANSFER - 2)) {
        ESP_LOGE(TAG, "WriteMulti of %lu bytes exceeds the %d byte buffer", (unsigned long)count, VL53L1_MAX_TRANSFER);
        return -1;
    }

    // 16-bit register index, most significant byte first, then the payload in one transaction.
    uint8_t buffer[VL53L1_MAX_TRANSFER];
    buffer[0] = (uint8_t)(index >> 8);
    buffer[1] = (uint8_t)(index & 0xFF);
    memcpy(&buffer[2], pdata, count);

    esp_err_t error = i2c_master_transmit(sensor_handle, buffer, count + 2, VL53L1_I2C_TIMEOUT_MS);
    return (error == ESP_OK) ? 0 : -1;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    (void)dev;

    if (sensor_handle == NULL || pdata == NULL) {
        return -1;
    }

    // Write the 16-bit index, then read back without releasing the bus (repeated start).
    const uint8_t index_bytes[2] = {
        (uint8_t)(index >> 8),
        (uint8_t)(index & 0xFF),
    };

    esp_err_t error = i2c_master_transmit_receive(sensor_handle,
                                                  index_bytes, sizeof(index_bytes),
                                                  pdata, count,
                                                  VL53L1_I2C_TIMEOUT_MS);
    return (error == ESP_OK) ? 0 : -1;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
    return VL53L1_WriteMulti(dev, index, &data, 1);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
    // Big endian on the wire, same as the register index.
    uint8_t buffer[2] = {
        (uint8_t)(data >> 8),
        (uint8_t)(data & 0xFF),
    };
    return VL53L1_WriteMulti(dev, index, buffer, sizeof(buffer));
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
    uint8_t buffer[4] = {
        (uint8_t)(data >> 24),
        (uint8_t)(data >> 16),
        (uint8_t)(data >> 8),
        (uint8_t)(data & 0xFF),
    };
    return VL53L1_WriteMulti(dev, index, buffer, sizeof(buffer));
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
    if (data == NULL) {
        return -1;
    }
    return VL53L1_ReadMulti(dev, index, data, 1);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
    if (data == NULL) {
        return -1;
    }

    uint8_t buffer[2] = {0};
    int8_t status = VL53L1_ReadMulti(dev, index, buffer, sizeof(buffer));
    if (status != 0) {
        return status;
    }

    *data = (uint16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
    return 0;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
    if (data == NULL) {
        return -1;
    }

    uint8_t buffer[4] = {0};
    int8_t status = VL53L1_ReadMulti(dev, index, buffer, sizeof(buffer));
    if (status != 0) {
        return status;
    }

    *data = ((uint32_t)buffer[0] << 24) |
            ((uint32_t)buffer[1] << 16) |
            ((uint32_t)buffer[2] << 8)  |
            ((uint32_t)buffer[3]);
    return 0;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms) {
    (void)dev;

    if (wait_ms <= 0) {
        return 0;
    }

    // This blocks the calling task. ST's api only calls it during init and during
    // calibration, never inside the ranging read path, so it never blocks the 100 Hz loop.
    vTaskDelay(pdMS_TO_TICKS(wait_ms));
    return 0;
}
