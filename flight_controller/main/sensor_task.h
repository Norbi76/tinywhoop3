#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "nav_estimator.h"

// The 100 Hz navigation sensor task.
//
// Deliberately kept off the 1 kHz control loop and off core 1. The ToF and the flow sensor are
// both slow blocking I/O (I2C clock stretching, SPI with mandatory microsecond delays) and
// letting either of them into fc_task would cause it to miss its 1 ms deadline.
//
// The task publishes a nav_state_t behind a mutex; fc_task takes a short-timeout copy of it
// once per iteration and carries on with stale data rather than blocking if the mutex is busy.

// Initialises the mutex and both navigation sensors.
// NEITHER SENSOR IS REQUIRED FOR FLIGHT: if the ToF or the flow sensor fails to initialise
// this logs a warning and carries on, and the corresponding nav validity flag simply stays
// false so flight_control disengages the outer loops. imu_setup() must have run first,
// because the ToF shares the IMU's I2C bus.
// @return ESP_OK if the mutex was created. Sensor failures are reported but not fatal.
esp_err_t sensor_task_init(void);

// Creates and starts the sensor task.
// @param core_id Which core to pin to. Should be core 0, opposite fc_task.
// @return ESP_OK on success, ESP_FAIL if the task could not be created.
esp_err_t sensor_task_start(BaseType_t core_id);

// Takes a mutex-protected copy of the latest navigation state.
// If the mutex cannot be taken quickly, `out` is left untouched and the function returns
// false - the caller should keep using its previous copy rather than stall the control loop.
// @param out Destination, must not be NULL.
// @return true if a fresh copy was taken, false if the mutex was busy.
bool sensor_task_get_nav_state(nav_state_t *out);
