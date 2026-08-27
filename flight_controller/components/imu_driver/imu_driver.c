// imu_driver.c - I2C bring-up, register configuration, sampling and calibration for the IMU.
//
// WHAT THIS FILE DOES
//   1. imu_setup()                  creates I2C0 (SDA 12 / SCL 11, 400 kHz), adds the device at
//                                   0x68, resets it, writes the range + DLPF configuration, and
//                                   confirms WHO_AM_I.
//   2. imu_read_raw_data()          one 14-byte burst from ACCEL_XOUT_H -> seven int16_t.
//   3. imu_convert_raw_to_physical() raw LSB -> g / dps / degC, gyro bias removed.
//   4. imu_calibrate_gyro/acc()     boot-time bias measurement (see each function).
//   5. imu_compute_roll_pitch()     stateless gravity-vector angles.
//
// HOW THE PIECES FIT
//   All mutable state is module-static: the two I2C handles and the four calibration offsets.
//   That makes the component single-instance and unsynchronised BY DESIGN - exactly one task
//   (fc_task) ever calls in here, so no lock is needed and none is paid for in the 1 kHz loop.
//
//   Only imu_read_raw_data() runs in the hot loop. Everything else blocks (infinite I2C timeout,
//   and the calibrations vTaskDelay for seconds) and is therefore startup-only.
//
// A NOTE ON THIS PARTICULAR SENSOR
//   MPU_9250_WHO_AM_I below is 0x74, not the 0x71 a genuine InvenSense part reports. This is a
//   clone, and its noise floor is several times the datasheet figure. That is why the gyro
//   calibration's motion threshold is set as loosely as it is - see GYRO_CALIB_MAX_SPREAD_LSB.

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

#define MPU_9250_WHO_AM_I 0x74

//register map...
#define REG_CONFIG        0x1A // Low-Pass Filter (DLPF) - gyro only
#define REG_GYRO_CONFIG   0x1B // Gyroscope full-scale range
#define REG_ACCEL_CONFIG  0x1C // Accelerometer full-scale range
#define REG_ACCEL_CONFIG2 0x1D // Accelerometer Low-Pass Filter (DLPF)
#define REG_ACCEL_XOUT_H  0x3B // Start address of the data burst (Y, Z and gyro follow)
#define REG_PWR_MGMT_1    0x6B // Power Management (Wake-up)
#define REG_WHO_AM_I      0x75 // Sensor identifier

#define RoomTemp_Offset 0.0f //per datasheet
#define Temp_Sensitivity  333.87f //per datasheet

#define Gyro_Sensitivity 16.4f
#define Acc_Sensitivity 4096.0f
//the imu has a 16-bit ADC -> it can represent up to 65535 values, i.e. [-32768, +32767]
//acc scale is set to +/-8g -> sensitivity = 32768 / 8 = 4096
//gyro scale is set to 2000dps -> sensitivity = 32768 / 2000 ~ 16.4

static float gyro_offset_x = 0.0f;
static float gyro_offset_y = 0.0f;
static float gyro_offset_z = 0.0f;

static float roll_offset = 0.0f;
static float pitch_offset = 0.0f;

static i2c_master_dev_handle_t imu_handle;
static i2c_master_bus_handle_t imu_bus_handle;

static esp_err_t imu_write_reg(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(imu_handle, write_buf, sizeof(write_buf), -1); //last param: timeout, -1 means wait forever
}

static esp_err_t imu_read_reg(uint8_t reg_addr, uint8_t *data_buffer, size_t len) {
    return i2c_master_transmit_receive(imu_handle, &reg_addr, 1, data_buffer, len, -1);
}

esp_err_t imu_setup(void) {
    if (imu_handle != NULL) {
        return ESP_OK;
    }
    //prevents initialising the i2c bus and the imu more than once

    //bus configuration
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA,
        .scl_io_num = I2C_MASTER_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7, //typical value, as given in the struct's documentation
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &imu_bus_handle));

    //sensor configuration
    i2c_device_config_t imu_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(imu_bus_handle, &imu_config, &imu_handle));

    ESP_ERROR_CHECK(imu_write_reg(REG_PWR_MGMT_1, 0x80)); // reset the imu
    vTaskDelay(pdMS_TO_TICKS(100)); // settling time after reset, per the datasheet
    ESP_ERROR_CHECK(imu_write_reg(REG_PWR_MGMT_1, 0x01)); // automatically select the best clock source
    ESP_ERROR_CHECK(imu_write_reg(REG_CONFIG, 0x03)); // DLPF at 41 Hz
    ESP_ERROR_CHECK(imu_write_reg(REG_GYRO_CONFIG, 0x18)); // +/- 2000dps
    ESP_ERROR_CHECK(imu_write_reg(REG_ACCEL_CONFIG, 0x10)); // +/- 8g

    // The REG_CONFIG write above filters ONLY the gyroscope. The accelerometer has its own DLPF,
    // in ACCEL_CONFIG_2, and its value after reset is 0x00 = 460 Hz bandwidth (register map, table
    // "Accelerometer Data Rates and Bandwidths"). Without the write below, the accelerometer ran
    // unfiltered at 460 Hz while the gyroscope ran at 41 Hz: propeller vibration goes straight
    // into the accelerometer, and the complementary filter - which trusts acceleration to show
    // where "down" is - gets noise instead of gravity, so the drone no longer holds its angles.
    // A_DLPF_CFG = 3 -> 41 Hz, 11.8 ms delay, i.e. the same bandwidth as the gyroscope.
    ESP_ERROR_CHECK(imu_write_reg(REG_ACCEL_CONFIG2, 0x03)); // accelerometer DLPF at 41 Hz

    uint8_t who_am_i = 0;
    esp_err_t error;
    if ((error = imu_read_reg(REG_WHO_AM_I, &who_am_i, 1)) != ESP_OK) {
        ESP_LOGE(TAG, "Can't find imu at adress 0x%x! imu_read_reg function returned %s!", IMU_ADDR, esp_err_to_name(error));
        return ESP_FAIL;
    }

    if (who_am_i != MPU_9250_WHO_AM_I) {
        ESP_LOGE(TAG, "Expected WHO_AM_I value: 0x%x, but got: 0x%x", MPU_9250_WHO_AM_I, who_am_i);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "IMU MPU 9250 found at address 0x%x", IMU_ADDR);
    

    return ESP_OK;
}

// The one function called from the 1 kHz loop.
//
// ACCEL_XOUT_H .. GYRO_ZOUT_L are 14 contiguous registers, so a single burst gets all seven
// values. Two reasons that matters: it is ~375 us of bus time at 400 kHz instead of three
// separate transactions (which would not fit the 1 ms tick), and all axes come from the same
// internal sample instant rather than three slightly different ones.
// Bytes arrive big-endian (high byte first) and are recombined here into signed 16-bit values.
esp_err_t imu_read_raw_data(imu_raw_data_t *data) {
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t raw_data[14]; //14 registers for acc, gyro and temp
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

// Number of samples per attempt. 1000 x 2 ms = 2 s of averaging, twice what it used to be: the
// bias estimation error falls with the square root of the sample count, and the Z bias is exactly
// the one that makes the drone rotate slowly without it ever "seeing" that it does.
#define GYRO_CALIB_SAMPLES 1000

// How much an axis is allowed to vary (raw LSB, peak to peak) during the window for us to accept
// that the IMU really did stay still. At 16.4 LSB/dps, 200 LSB ~ 12 dps.
//
// The threshold is deliberately loose. Peak-to-peak over 1000 samples is ~6.6 sigma, and this
// sensor reports WHO_AM_I = 0x74, so it is not a genuine InvenSense part - a clone's noise is
// several times the datasheet figure. A tight threshold would raise false alarms on every boot,
// which is worse than the check that was missing before. A drone held in the hand shows hundreds
// of LSB, so 200 still catches that.
//
// TODO(bench): the message at the end of calibration prints the measured "worst spread". Look at
// it on your own hardware after a few boots and tighten the threshold to ~3x the typical value.
#define GYRO_CALIB_MAX_SPREAD_LSB 200.0f

#define GYRO_CALIB_MAX_ATTEMPTS 3

// Measures the gyro's zero-rate bias by averaging 2 s of samples, and REJECTS the window if the
// drone moved during it (see the spread check inline below). Up to 3 attempts.
//
// If every attempt sees motion it logs an error and returns with whatever offsets it had, rather
// than blocking boot: a bootable drone plus a loud warning beats a hang on the bench.
void imu_calibrate_gyro(void) {
    imu_raw_data_t raw_data;

    for (int attempt = 1; attempt <= GYRO_CALIB_MAX_ATTEMPTS; attempt++) {
        ESP_LOGI(TAG, "Calibrating gyro (attempt %d/%d)... Please keep the IMU stationary.",
                 attempt, GYRO_CALIB_MAX_ATTEMPTS);

        float sum[3] = {0.0f, 0.0f, 0.0f};
        float minimum[3] = {0.0f}, maximum[3] = {0.0f};
        int good_samples = 0;

        for (int i = 0; i < GYRO_CALIB_SAMPLES; i++) {
            if (imu_read_raw_data(&raw_data) == ESP_OK) {
                const float axis[3] = {
                    (float)raw_data.gyro_x_raw,
                    (float)raw_data.gyro_y_raw,
                    (float)raw_data.gyro_z_raw,
                };

                for (int a = 0; a < 3; a++) {
                    sum[a] += axis[a];
                    if (good_samples == 0 || axis[a] < minimum[a]) minimum[a] = axis[a];
                    if (good_samples == 0 || axis[a] > maximum[a]) maximum[a] = axis[a];
                }
                good_samples++;
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }

        if (good_samples == 0) {
            ESP_LOGE(TAG, "Gyro calibration read no samples at all - I2C bus problem?");
            continue;
        }

        // --- Motion check ----------------------------------------------------
        // Averaging a window in which the drone was moved produces a plausible-looking offset
        // that is simply wrong, and the old code did that silently. A wrong Z offset is
        // indistinguishable in flight from a drone that slowly rotates on its own, because the
        // controller subtracts the bad offset and concludes the yaw rate is zero.
        float worst_spread = 0.0f;
        int worst_axis = 0;
        for (int a = 0; a < 3; a++) {
            const float spread = maximum[a] - minimum[a];
            if (spread > worst_spread) {
                worst_spread = spread;
                worst_axis = a;
            }
        }

        if (worst_spread > GYRO_CALIB_MAX_SPREAD_LSB) {
            ESP_LOGW(TAG, "Gyro moved during calibration (axis %c spread %.0f LSB = %.1f dps, limit %.0f) - retrying",
                     "XYZ"[worst_axis], worst_spread, worst_spread / Gyro_Sensitivity,
                     GYRO_CALIB_MAX_SPREAD_LSB);
            continue;
        }

        gyro_offset_x = sum[0] / good_samples;
        gyro_offset_y = sum[1] / good_samples;
        gyro_offset_z = sum[2] / good_samples;

        ESP_LOGI(TAG, "Gyro calibration complete: offsets - X: %.2f, Y: %.2f, Z: %.2f (%.2f dps on Z, worst spread %.0f LSB)",
                 gyro_offset_x, gyro_offset_y, gyro_offset_z,
                 gyro_offset_z / Gyro_Sensitivity, worst_spread);
        return;
    }

    // Every attempt saw motion. Carrying on with whatever offsets we have is still better than
    // never finishing boot - but say so loudly, because yaw will drift and roll/pitch will lean.
    ESP_LOGE(TAG, "Gyro calibration FAILED after %d attempts - the drone was never still. "
                  "Offsets left at X: %.2f, Y: %.2f, Z: %.2f. Reboot on a stable surface.",
             GYRO_CALIB_MAX_ATTEMPTS, gyro_offset_x, gyro_offset_y, gyro_offset_z);
}

// Measures the IMU's MOUNTING error, not a sensor bias: the roll/pitch that the gravity vector
// reports while the drone is sitting level. imu_compute_roll_pitch() subtracts it, so "level on
// the bench" reads as 0/0 even if the IMU is not perfectly square to the frame.
//
// Deliberately has no motion check, unlike the gyro routine. A bad accel calibration shows up as
// a steady lean the pilot can see immediately; a bad gyro bias shows up as an invisible slow yaw.
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