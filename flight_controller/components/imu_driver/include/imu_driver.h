#pragma once //previne includerea multipla a fisierului
#include "esp_err.h"
#include <stdint.h>

typedef struct {
    int16_t acc_x, acc_y, acc_z, 
            gyro_x, gyro_y, gyro_z;
} imu_raw_data_t;

esp_err_t imu_setup(void);
esp_err_t imu_read_raw_data(imu_raw_data_t *data);
//functie pentru calibrare