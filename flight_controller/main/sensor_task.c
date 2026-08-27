// sensor_task.c - the 100 Hz navigation task and the nav_state_t handoff to the control loop.
//
// WHAT THIS FILE DOES
//   Polls the two slow navigation sensors, feeds nav_estimator, and publishes the result behind
//   a mutex that fc_task samples with a zero timeout.
//
// HOW THE CADENCE IS CHOSEN
//   Task runs at 100 Hz. Optical flow is read every cycle; the ToF only every 3rd (~33 Hz),
//   because it is configured for a 25 ms inter-measurement period and polling faster would just
//   return "not ready" while occupying an I2C bus the 1 kHz IMU loop is also using.
//
// THE NULL CONVENTION
//   A failed sensor read still calls nav_estimator_update_*(), with NULL. That is how the
//   estimator is told "no data this cycle" so it can run its own 200 ms timeout and eventually
//   drop the validity flag - rather than silently freezing the last good altitude forever.
//
// THE MUTEX ASYMMETRY IS THE DESIGN
//   Writer (this task, 100 Hz) takes it with a 2 ms timeout and skips a publish if busy.
//   Reader (fc_task, 1 kHz) takes it with a ZERO timeout and keeps its previous copy if busy.
//   The 1 kHz side must never wait; the 100 Hz side can afford 2 ms out of its 10 ms budget.
//
// NEITHER SENSOR IS REQUIRED FOR FLIGHT - sensor_task_init() warns and continues on either
// failure, and only returns an error if the MUTEX could not be created.

#include "sensor_task.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "attitude_estimator.h"
#include "vl53l1x_driver.h"
#include "pmw3901_driver.h"

static const char *TAG = "SENSOR_TASK";

#define SENSOR_TASK_HZ 100
#define SENSOR_TASK_PERIOD_MS (1000 / SENSOR_TASK_HZ)

// The ToF is configured for a 25 ms inter-measurement period (~40 Hz), so polling it on every
// 100 Hz cycle would mostly return "not ready". Every 3rd cycle is ~33 Hz, just under the
// sensor's own rate, which keeps the I2C bus free for the IMU.
#define TOF_READ_DIVIDER 3

#define SENSOR_TASK_STACK_SIZE 4096
#define SENSOR_TASK_PRIORITY 6

static SemaphoreHandle_t nav_state_mutex;
static nav_state_t shared_nav_state;

esp_err_t sensor_task_init(void) {
    nav_state_mutex = xSemaphoreCreateMutex();
    if (nav_state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create nav state mutex");
        return ESP_FAIL;
    }

    esp_err_t error = nav_estimator_init(1.0f / ((float)SENSOR_TASK_HZ / (float)TOF_READ_DIVIDER));
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "nav_estimator_init failed: %s", esp_err_to_name(error));
        return error;
    }

    // --- ToF ---------------------------------------------------------------
    // Not fatal. Without it there is no altitude, so altitude hold and position hold stay
    // unavailable, but angle mode is completely unaffected.
    error = vl53l1x_init();
    if (error != ESP_OK) {
        ESP_LOGW(TAG, "VL53L1X unavailable (%s) - altitude hold disabled, angle mode unaffected",
                 esp_err_to_name(error));
    }

    // --- Optical flow -------------------------------------------------------
    // Also not fatal. Without it there is no body velocity, so position hold stays unavailable.
    error = pmw3901_init();
    if (error != ESP_OK) {
        ESP_LOGW(TAG, "PMW3901 unavailable (%s) - position hold disabled, angle mode unaffected",
                 esp_err_to_name(error));
    }

    return ESP_OK;
}

static void sensor_task(void *args) {
    ESP_LOGI(TAG, "Sensor task started on core %d at %d Hz", xPortGetCoreID(), SENSOR_TASK_HZ);

    const TickType_t period = pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS);
    TickType_t last_wake_time = xTaskGetTickCount();

    uint32_t cycle = 0;
    int64_t last_range_us = esp_timer_get_time();
    int64_t last_flow_us = last_range_us;

    for (;;) {
        attitude_state_t attitude;
        attitude_get(&attitude);

        const int64_t now_us = esp_timer_get_time();

        // --- ToF, every 3rd cycle (~33 Hz) ---------------------------------
        if ((cycle % TOF_READ_DIVIDER) == 0) {
            const float range_dt = (float)(now_us - last_range_us) / 1000000.0f;
            last_range_us = now_us;

            vl53l1x_result_t range;
            if (vl53l1x_read(&range) == ESP_OK) {
                nav_estimator_update_range(&range, attitude.roll, attitude.pitch, range_dt);
            } else {
                // Pass NULL so the estimator can run its own timeout logic and eventually
                // drop altitude_valid, rather than silently freezing the last good altitude.
                nav_estimator_update_range(NULL, attitude.roll, attitude.pitch, range_dt);
            }
        }

        // --- Optical flow, every cycle (100 Hz) -----------------------------
        {
            const float flow_dt = (float)(now_us - last_flow_us) / 1000000.0f;
            last_flow_us = now_us;

            pmw3901_motion_t flow;
            if (pmw3901_read_motion(&flow) == ESP_OK) {
                nav_estimator_update_flow(&flow, attitude.roll_rate, attitude.pitch_rate, flow_dt);
            } else {
                nav_estimator_update_flow(NULL, attitude.roll_rate, attitude.pitch_rate, flow_dt);
            }
        }

        // --- Publish --------------------------------------------------------
        nav_state_t local_state;
        nav_estimator_get(&local_state);

        if (xSemaphoreTake(nav_state_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            shared_nav_state = local_state;
            xSemaphoreGive(nav_state_mutex);
        }

        cycle++;
        vTaskDelayUntil(&last_wake_time, period);
    }
}

esp_err_t sensor_task_start(BaseType_t core_id) {
    BaseType_t result = xTaskCreatePinnedToCore(
        sensor_task,
        "SENSOR_TASK",
        SENSOR_TASK_STACK_SIZE,
        NULL,
        SENSOR_TASK_PRIORITY,
        NULL,
        core_id);

    return (result == pdPASS) ? ESP_OK : ESP_FAIL;
}

bool sensor_task_get_nav_state(nav_state_t *out) {
    if (out == NULL || nav_state_mutex == NULL) {
        return false;
    }

    // Zero timeout: the 1 kHz control loop must never block waiting on the 100 Hz sensor task.
    // A missed copy just means fc_task reuses last iteration's nav state, which is at most
    // 1 ms stale and completely harmless for a 50 Hz outer loop.
    if (xSemaphoreTake(nav_state_mutex, 0) != pdTRUE) {
        return false;
    }

    *out = shared_nav_state;
    xSemaphoreGive(nav_state_mutex);

    return true;
}
