#include "imu_driver.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"

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

#define RoomTemp_Offset 0.0f //conform datasheet
#define Temp_Sensitivity  333.87f //conform datasheet

#define Gyro_Sensitivity 16.4f
#define Acc_Sensitivity 4096.0f 
//imu are un ADC pe 16 biti -> poate reprezenta valori pana 65535 sau [-32768, +32767]
//acc are scala setata la +/-8g -> sensibilitatea = 32768 / 8 = 4096
//gyro are scala setata la 2000dps -> sensibilitatea = 32768 / 2000 ~ 16.4

static float gyro_offset_x = 0.0f;
static float gyro_offset_y = 0.0f;
static float gyro_offset_z = 0.0f;

static float roll_offset = 0.0f;
static float pitch_offset = 0.0f;

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

    ESP_ERROR_CHECK(imu_write_reg(REG_PWR_MGMT_1, 0x80)); // reseteaza imu
    vTaskDelay(pdMS_TO_TICKS(100)); // timp de asteptare dupa reset conform datasheet ului 
    ESP_ERROR_CHECK(imu_write_reg(REG_PWR_MGMT_1, 0x01)); // selectarea automat cea mai buna sursa de clk
    ESP_ERROR_CHECK(imu_write_reg(REG_CONFIG, 0x03)); // DLPF la 41 Hz
    ESP_ERROR_CHECK(imu_write_reg(REG_GYRO_CONFIG, 0x18)); // +/- 2000dps
    ESP_ERROR_CHECK(imu_write_reg(REG_ACCEL_CONFIG, 0x10)); // +/- 8g

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

    uint8_t raw_data[14]; //14 registrii pentru acc, gyto si temp
    esp_err_t error = imu_read_reg(REG_ACCEL_XOUT_H, raw_data, sizeof(raw_data));
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read IMU data: %s", esp_err_to_name(error));
        return error;
    }

    data->acc_x_raw = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    data->acc_y_raw = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    data->acc_z_raw = (int16_t)((raw_data[4] << 8) | raw_data[5]);
    data->temp_raw = (int16_t)((raw_data[6] << 8) | raw_data[7]);
    data->gyro_x_raw = (int16_t)((raw_data[8] << 8) | raw_data[9]);
    data->gyro_y_raw = (int16_t)((raw_data[10] << 8) | raw_data[11]);
    data->gyro_z_raw = (int16_t)((raw_data[12] << 8) | raw_data[13]);

    return ESP_OK;
}

void imu_convert_raw_to_physical(imu_raw_data_t *raw_data, imu_physical_data_t *physical_data) {
    physical_data->acc_x_g = raw_data->acc_x_raw / Acc_Sensitivity;
    physical_data->acc_y_g = raw_data->acc_y_raw / Acc_Sensitivity;
    physical_data->acc_z_g = raw_data->acc_z_raw / Acc_Sensitivity;

    physical_data->temp_C = ((raw_data->temp_raw + RoomTemp_Offset) / Temp_Sensitivity) + 21;

    physical_data->gyro_x_dps = (raw_data->gyro_x_raw - gyro_offset_x) / Gyro_Sensitivity;
    physical_data->gyro_y_dps = (raw_data->gyro_y_raw - gyro_offset_y) / Gyro_Sensitivity;
    physical_data->gyro_z_dps = (raw_data->gyro_z_raw - gyro_offset_z) / Gyro_Sensitivity;
}

void imu_calibrate_gyro(void) {
    ESP_LOGI(TAG, "Calibrating gyro... Please keep the IMU stationary.");
    imu_raw_data_t raw_data;
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    const int num_samples = 500;

    for (int i = 0; i < num_samples; i++) {
        if (imu_read_raw_data(&raw_data) == ESP_OK) {
            sum_x += raw_data.gyro_x_raw;
            sum_y += raw_data.gyro_y_raw;
            sum_z += raw_data.gyro_z_raw;
        }
        vTaskDelay(pdMS_TO_TICKS(2)); 
    }

    gyro_offset_x = sum_x / num_samples;
    gyro_offset_y = sum_y / num_samples;
    gyro_offset_z = sum_z / num_samples;

    ESP_LOGI(TAG, "Gyro calibration complete: offsets - X: %.2f, Y: %.2f, Z: %.2f", gyro_offset_x, gyro_offset_y, gyro_offset_z);
}

void imu_calibrate_acc(void) {
    ESP_LOGI(TAG, "Calibrating accelerometer... Please keep the IMU stationary and level.");
    imu_raw_data_t raw_data;
    imu_physical_data_t physical_data;
    float roll_sum = 0.0f, pitch_sum = 0.0f;
    const int num_samples = 500;

    for (int i = 0; i < num_samples; i++) {
        imu_read_raw_data(&raw_data);

        imu_convert_raw_to_physical(&raw_data, &physical_data);

        roll_sum += atan2(physical_data.acc_y_g, physical_data.acc_z_g) * (180.0 / M_PI);
        pitch_sum += atan2(-physical_data.acc_x_g, 
                           sqrt(physical_data.acc_y_g * physical_data.acc_y_g + 
                                physical_data.acc_z_g * physical_data.acc_z_g)) * (180.0 / M_PI);

        vTaskDelay(pdMS_TO_TICKS(2)); 
    }

    roll_offset = roll_sum / num_samples;
    pitch_offset = pitch_sum / num_samples;

    ESP_LOGI(TAG, "Accelerometer calibration complete: roll offset: %.2f, pitch offset: %.2f", roll_offset, pitch_offset);
}

void imu_compute_roll_pitch(float acc_x_g, float acc_y_g, float acc_z_g, float *roll_angle, float *pitch_angle) {
    if (roll_angle == NULL || pitch_angle == NULL) {
        ESP_LOGE(TAG, "Invalid pointers for roll or pitch.");
        return;
    }

    *roll_angle = (atan2(acc_y_g, acc_z_g) * (180.0 / M_PI)) - roll_offset;
    *pitch_angle = (atan2(-acc_x_g, sqrt(acc_y_g * acc_y_g + acc_z_g * acc_z_g)) * (180.0 / M_PI)) - pitch_offset;
}