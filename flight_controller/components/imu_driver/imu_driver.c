#include "imu_driver.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "IMU";

#define I2C_MASTER_SDA 12
#define I2C_MASTER_SCL 11

#define I2C_MASTER_FREQ_HZ 400000
#define IMU_ADDR 0x68

#define MPU_6500_WHO_AM_I 0x70

//harta registrilor...
#define REG_CONFIG       0x1A // Filtrul Low-Pass (DLPF)
#define REG_GYRO_CONFIG  0x1B // Scala Giroscopului
#define REG_ACCEL_CONFIG 0x1C // Scala Accelerometrului
#define REG_ACCEL_XOUT_H 0x3B // Adresa de start a datelor (urmează Y, Z și Gyro)
#define REG_PWR_MGMT_1   0x6B // Power Management (Wake-up)
#define REG_WHO_AM_I     0x75 // Identificatorul senzorului

static i2c_master_dev_handle_t imu_handle;
static i2c_master_bus_handle_t imu_bus_handle;

static esp_err_t imu_write_reg(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(imu_handle, write_buf, sizeof(write_buf), -1); //ultimul param: timeout, -1 inseamna ca se asteapta la nesfarsit
}

static esp_err_t imu_read_reg(uint8_t reg_addr, uint8_t *data_buffer, size_t len) {
    return i2c_master_transmit_receive(imu_handle, &reg_addr, 1, data_buffer, len, -1);
}

esp_err_t imu_setup(void) {
    if (imu_handle != NULL) {
        return ESP_OK;
    }
    //previne initializarea multipla a magistralei i2c si al senzorului imu

    //configurarea magistralei
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA,
        .scl_io_num = I2C_MASTER_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7, //valoare tipica utilizata mentionata in documentatia structurii
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &imu_bus_handle));

    //configurarea senzorului
    i2c_device_config_t imu_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(imu_bus_handle, &imu_config, &imu_handle));

    ESP_ERROR_CHECK(imu_write_reg(REG_PWR_MGMT_1, 0x80));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(imu_write_reg(REG_PWR_MGMT_1, 0x01));
    ESP_ERROR_CHECK(imu_write_reg(REG_CONFIG, 0x03));
    ESP_ERROR_CHECK(imu_write_reg(REG_GYRO_CONFIG, 0x00));
    ESP_ERROR_CHECK(imu_write_reg(REG_ACCEL_CONFIG, 0x00));

    uint8_t who_am_i = 0;
    esp_err_t error;
    if ((error = imu_read_reg(REG_WHO_AM_I, &who_am_i, 1)) != ESP_OK) {
        ESP_LOGE(TAG, "Can't find imu at adress 0x%x! imu_read_reg function returned %s!", IMU_ADDR, esp_err_to_name(error));
        return ESP_FAIL;
    }

    if (who_am_i != MPU_6500_WHO_AM_I) {
        ESP_LOGE(TAG, "Expected WHO_AM_I value: 0x%x, but got: 0x%x", MPU_6500_WHO_AM_I, who_am_i);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "IMU MPU 6500 found at address 0x%x", IMU_ADDR);
    

    return ESP_OK;
}

esp_err_t imu_read_raw_data(imu_raw_data_t *data) {
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t raw_data[14];
    esp_err_t error = imu_read_reg(REG_ACCEL_XOUT_H, raw_data, sizeof(raw_data));
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read IMU data: %s", esp_err_to_name(error));
        return error;
    }

    data->acc_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    data->acc_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    data->acc_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
    data->gyro_x = (int16_t)((raw_data[8] << 8) | raw_data[9]);
    data->gyro_y = (int16_t)((raw_data[10] << 8) | raw_data[11]);
    data->gyro_z = (int16_t)((raw_data[12] << 8) | raw_data[13]);

    return ESP_OK;
}