#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "imu_driver.h"
#include "esp_timer.h"
#include "telemetry_uart.h"

#include "attitude_estimator.h"
#include "nav_estimator.h"
#include "motor_driver.h"
#include "flight_control.h"
#include "sensor_task.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "FC_MAIN";
// #define ALPHA 0.70

static SemaphoreHandle_t imu_data_mutex;
static telemetry_imu_payload_t latest_imu_payload;

// Signalled by fc_task once the IMU is up and calibrated. app_main waits on this before
// initialising the navigation sensors, because the VL53L1X shares the I2C bus that
// imu_setup() creates and must not touch it first.
static SemaphoreHandle_t imu_ready_semaphore;

void fc_task(void *args) { //semnatura unui task freeRTOS trebuie sa contina un param de tip void*
    ESP_LOGI(TAG, "Flight controll task started on core %d", xPortGetCoreID());
    //init imu, pwm
    esp_err_t error = imu_setup();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "IMU setup failed: %s", esp_err_to_name(error));
        vTaskDelete(NULL);
        return;
    }
    // float roll_offset = 0.0f, pitch_offset = 0.0f;
    imu_calibrate_acc();
    imu_calibrate_gyro();

    // The IMU is up and the I2C bus exists - app_main can now bring up the ToF and the flow
    // sensor without racing us for the bus.
    if (imu_ready_semaphore != NULL) {
        xSemaphoreGive(imu_ready_semaphore);
    }

    // 1 tick == 1 ms because CONFIG_FREERTOS_HZ is 1000 (see sdkconfig.defaults).
    // NOTE: this used to be pdMS_TO_TICKS(1000), which is 1000 ms, so the "1 kHz" loop was
    // actually running at 1 Hz. Raising the tick rate is what makes 1 kHz reachable at all.
    const TickType_t xFreq = pdMS_TO_TICKS(1); // 1ms -> 1kHz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    int64_t last_time = esp_timer_get_time();


    imu_raw_data_t imu_raw_data;
    imu_physical_data_t imu_physical_data;

    float roll_angle = 0.0f, pitch_angle = 0.0f;
    // float roll_angle_comp = 0.0f, pitch_angle_comp = 0.0f;

    attitude_state_t attitude;
    nav_state_t nav_state = {0};   // retained between iterations if the mutex is busy

    for (;;) {
        error = imu_read_raw_data(&imu_raw_data);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "IMU read failed: %s", esp_err_to_name(error));
            // A dead IMU means no attitude, which means the drone must not be flying.
            motor_all_stop();
            vTaskDelay(pdMS_TO_TICKS(10)); // wait a bit before retrying
            continue;
        }
        imu_convert_raw_to_physical(&imu_raw_data, &imu_physical_data);


        const int64_t current_time = esp_timer_get_time();
        float dt = (float)(current_time - last_time) / 1000000.0f; // convert to seconds
        last_time = current_time;

        // Guard against a silly dt after a scheduling hiccup - a huge dt would produce a huge
        // derivative and integrator step in the cascade.
        if (dt <= 0.0f || dt > 0.02f) {
            dt = 1.0f / 1000.0f;
        }

        imu_compute_roll_pitch(imu_physical_data.acc_x_g, imu_physical_data.acc_y_g, imu_physical_data.acc_z_g, &roll_angle, &pitch_angle);

        if (imu_data_mutex != NULL && xSemaphoreTake(imu_data_mutex, 0) == pdTRUE) {
            latest_imu_payload.acc_x = imu_physical_data.acc_x_g;
            latest_imu_payload.acc_y = imu_physical_data.acc_y_g;
            latest_imu_payload.acc_z = imu_physical_data.acc_z_g;
            latest_imu_payload.gyro_x = imu_physical_data.gyro_x_dps;
            latest_imu_payload.gyro_y = imu_physical_data.gyro_y_dps;
            latest_imu_payload.gyro_z = imu_physical_data.gyro_z_dps;
            latest_imu_payload.roll = roll_angle;
            latest_imu_payload.pitch = pitch_angle;
            latest_imu_payload.temperature = imu_physical_data.temp_C;
            xSemaphoreGive(imu_data_mutex);
        }
        // roll_angle_comp = ALPHA * (roll_angle_comp + imu_physical_data.gyro_x_dps * dt) + (1 - ALPHA) * roll_angle;
        // pitch_angle_comp = ALPHA * (pitch_angle_comp + imu_physical_data.gyro_y_dps * dt) + (1 - ALPHA) * pitch_angle;

        // 1. Fuse accelerometer and gyro into an attitude estimate.
        attitude_update(&imu_physical_data, dt);
        attitude_get(&attitude);

        // 2. Pick up the latest navigation state from sensor_task. Zero-timeout: if the mutex
        //    is busy we keep last iteration's copy rather than blocking the 1 kHz loop.
        sensor_task_get_nav_state(&nav_state);

        // 3. Run the cascade, the arming state machine and the mixer. This writes the motors,
        //    including forcing them to zero on every disarmed path.
        flight_control_update(&attitude, &nav_state, dt);

        // Așteaptă exact până la următoarea milisecundă
        vTaskDelayUntil(&xLastWakeTime, xFreq);
    }
}

void telemetry_task(void *args) {
    //init uart...
    ESP_LOGI(TAG, "Telemtry transmission task started on core %d", xPortGetCoreID());

    telemetry_message_t rx_message;
    uint32_t cycle = 0;

    for (;;) {
        // --- Status frame, every cycle (50 Hz) -----------------------------
        // This is what the dashboard reads back: attitude, nav, motors, arming and the
        // validity flags.
        telemetry_status_payload_t status;
        flight_control_get_status(&status);

        esp_err_t send_error = uart_telemetry_send_message(
            TELEMETRY_MSG_STATUS,
            &status,
            sizeof(status)
        );
        if (send_error != ESP_OK) {
            ESP_LOGE(TAG, "Status telemetry send failed: %s", esp_err_to_name(send_error));
        }

        // --- Raw IMU frame, every 5th cycle (10 Hz) ------------------------
        // Not needed for control; kept for logging and for the raw-data view.
        if ((cycle % 5) == 0) {
            telemetry_imu_payload_t imu_payload = {0};

            if (imu_data_mutex != NULL && xSemaphoreTake(imu_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                imu_payload = latest_imu_payload;
                xSemaphoreGive(imu_data_mutex);
            }

            send_error = uart_telemetry_send_message(
                TELEMETRY_MSG_IMU,
                &imu_payload,
                sizeof(imu_payload)
            );
            if (send_error != ESP_OK) {
                ESP_LOGE(TAG, "IMU telemetry send failed: %s", esp_err_to_name(send_error));
            }
        }

        // --- Receive ---------------------------------------------------------
        // Drain everything queued rather than taking one frame per 20 ms cycle, otherwise the
        // RX buffer backs up and the control frames we act on get progressively staler.
        // The resync reader is used here: it discards one bad frame instead of flushing the
        // whole buffer, which matters because a flush would drop good control frames and
        // trip the 300 ms link watchdog.
        while (uart_telemetry_read_message_resync(&rx_message, 2)) {
            switch ((telemetry_msg_type_t)rx_message.header.msg_type) {
                case TELEMETRY_MSG_CONTROL: {
                    if (rx_message.header.payload_len < sizeof(telemetry_control_payload_t)) {
                        ESP_LOGW(TAG, "Short control frame: %u bytes", rx_message.header.payload_len);
                        break;
                    }
                    const telemetry_control_payload_t *control =
                        (const telemetry_control_payload_t *)rx_message.payload;

                    // Feeds the controller AND pets the link watchdog.
                    flight_control_set_control_input(control);
                    break;
                }
                case TELEMETRY_MSG_GAINS: {
                    if (rx_message.header.payload_len < sizeof(telemetry_gains_payload_t)) {
                        ESP_LOGW(TAG, "Short gains frame: %u bytes", rx_message.header.payload_len);
                        break;
                    }
                    const telemetry_gains_payload_t *gains =
                        (const telemetry_gains_payload_t *)rx_message.payload;

                    esp_err_t gain_error = flight_control_set_gains(
                        (telemetry_loop_id_t)gains->loop_id, gains->kp, gains->ki, gains->kd);
                    if (gain_error != ESP_OK) {
                        ESP_LOGW(TAG, "Rejected gains for loop %u", gains->loop_id);
                    }
                    break;
                }
                default:
                    // Other message types are not expected on this direction of the link.
                    break;
            }
        }

        cycle++;
        vTaskDelay(pdMS_TO_TICKS(20));
    }

}

void app_main(void)
{
    ESP_LOGI(TAG, "System starting up...");

    esp_err_t error;

    imu_data_mutex = xSemaphoreCreateMutex();
    if (imu_data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create IMU data mutex");
        return;
    }

    imu_ready_semaphore = xSemaphoreCreateBinary();
    if (imu_ready_semaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create IMU ready semaphore");
        return;
    }

    // --- Motors first -------------------------------------------------------
    // Before anything else can command thrust, get the LEDC channels configured and driven to
    // zero. Note this does NOT protect against the gates floating between reset and this
    // line - that needs physical pull-downs, see the TODO in motor_driver.c.
    error = motor_driver_init();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Motor driver init failed: %s", esp_err_to_name(error));
        return;
    }

    // --- Attitude estimator --------------------------------------------------
    // tau = 0.5 s: the gyro carries the short-term response, the accelerometer trims the
    // long-term drift out.
    error = attitude_init(1.0f / 1000.0f, 0.5f);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Attitude estimator init failed: %s", esp_err_to_name(error));
        return;
    }

    // --- Flight control ------------------------------------------------------
    error = flight_control_init();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Flight control init failed: %s", esp_err_to_name(error));
        return;
    }

    // --- Telemetry link ------------------------------------------------------
    telemetry_uart_config_t uart_config = {
        .uart_port  = UART_NUM_1,
        .tx_pin     = GPIO_NUM_6,   // TODO(pins): confirm against your wiring
        .rx_pin     = GPIO_NUM_7,   // TODO(pins): confirm against your wiring
        .baud_rate  = 460800
    };

    error = uart_telemetry_init(&uart_config);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize UART telemetry: %s", esp_err_to_name(error));
        return;
    }
    else {
        ESP_LOGI(TAG, "UART telemetry initialized successfully.");
    }

    // --- Control loop --------------------------------------------------------
    xTaskCreatePinnedToCore(
        fc_task,
        "Flight_Ctrl_Task",
        4096,
        NULL,
        configMAX_PRIORITIES - 1, //prioritatea cea mai mare, pentru a nu fi intrerupt de alte taskuri
        NULL,
        1
    );
    //de ce am ales core 1 pt task ul FC?
    //Core 0 -> PRO_CPU(PROTOCOL CPU): ruleaza multe task uri de "fundal" cum ar fi accese la mem, alte functii legate de os
    //Core 1 -> APP_CPU(APPLICATION CPU): este lasat mai liber

    // --- Navigation sensors --------------------------------------------------
    // Wait for fc_task to finish imu_setup() and the calibration sweeps, because the VL53L1X
    // attaches to the I2C bus that imu_setup() creates. Calibration is 500 samples at 2 ms on
    // each of two axes, so ~2 s; 10 s of headroom is plenty.
    if (xSemaphoreTake(imu_ready_semaphore, pdMS_TO_TICKS(10000)) != pdTRUE) {
        ESP_LOGE(TAG, "IMU never became ready - navigation sensors will not be started");
    } else {
        error = sensor_task_init();
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "Sensor task init failed: %s", esp_err_to_name(error));
        } else {
            error = sensor_task_start(0);
            if (error != ESP_OK) {
                ESP_LOGE(TAG, "Failed to start sensor task: %s", esp_err_to_name(error));
            }
        }
    }

    xTaskCreatePinnedToCore(
        telemetry_task,
        "Telemetry_Task",
        4096,
        NULL,
        5,
        NULL,
        0
    );
}
