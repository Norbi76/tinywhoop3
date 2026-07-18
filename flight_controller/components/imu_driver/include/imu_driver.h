#pragma once //previne includerea multipla a fisierului
#include "esp_err.h"
#include <stdint.h>

// Represents the raw data read from the IMU sensor, including accelerometer, gyroscope, and temperature values.
typedef struct {
    int16_t acc_x_raw, acc_y_raw, acc_z_raw, 
            gyro_x_raw, gyro_y_raw, gyro_z_raw,
            temp_raw;
} imu_raw_data_t;

// Represents the physical data computed based on raw data, including accelerometer, gyroscope, and temperature values.
typedef struct {
    float acc_x_g, acc_y_g, acc_z_g, 
            gyro_x_dps, gyro_y_dps, gyro_z_dps,
            temp_C;
} imu_physical_data_t;

//Sets up the IMU sensor(cutoff freq for DLPF, gyro and acc range, etc...) and I2C bus. 
//@return ESP_OK on success, or an error code on failure.
esp_err_t imu_setup(void);

//Reads all raw data from IMU sensor(accelerometer, gyroscope, temperature) and stores it in the provided imu_raw_data_t structure.
//@param data: Pointer to an imu_raw_data_t structure where the raw data will be stored
//@return ESP_OK on success, or an error code on failure.
esp_err_t imu_read_raw_data(imu_raw_data_t *data);

//Converts raw IMU data to physical units (g for accelerometer, degrees per second for gyroscope, and Celsius for temperature).
//@param raw_data: Pointer to an imu_raw_data_t structure containing the raw data
//@param physical_data: Pointer to an imu_physical_data_t structure where the converted data will be stored
//@return none: The converted data is stored in the imu_physical_data_t structure.
void imu_convert_raw_to_physical(imu_raw_data_t *raw_data, imu_physical_data_t *physical_data);

//Calibrates the gyroscope by averaging 500 samples while the IMU is stationary.
//@return none: The calculated offsets are stored in static variables.
void imu_calibrate_gyro(void);

//Calibrates the accelerometer by averaging 500 samples while the IMU is stationary and level.
//@return none: The calculated roll and pitch offsets are stored in static variables.
void imu_calibrate_acc(void);

//Computes roll and pitch angles from accelerometer data using the atan2 function.
//@param acc_x_g: Accelerometer X-axis data in g
//@param acc_y_g: Accelerometer Y-axis data in g
//@param acc_z_g: Accelerometer Z-axis data in g
//@param roll_angle: Pointer to a float where the computed roll angle will be stored
//@param pitch_angle: Pointer to a float where the computed pitch angle will be stored
//@return none: The computed roll and pitch angles are stored in the provided pointers.
void imu_compute_roll_pitch(float acc_x_g, float acc_y_g, float acc_z_g, float *roll, float *pitch);