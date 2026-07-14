#pragma once //previne includerea multipla a fisierului
#include "esp_err.h"
#include <stdint.h>

typedef struct {
    int16_t acc_x_raw, acc_y_raw, acc_z_raw, 
            gyro_x_raw, gyro_y_raw, gyro_z_raw,
            temp_raw;
} imu_raw_data_t;

typedef struct {
    float acc_x_g, acc_y_g, acc_z_g, 
            gyro_x_dps, gyro_y_dps, gyro_z_dps,
            temp_C;
} imu_physical_data_t;

esp_err_t imu_setup(void);
esp_err_t imu_read_raw_data(imu_raw_data_t *data);
void imu_convert_raw_to_physical(imu_raw_data_t *raw_data, imu_physical_data_t *physical_data);
void imu_calibrate_gyro(void);