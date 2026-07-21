#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "imu_driver.h"
#include "esp_timer.h"
#include "telemetry_uart.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "FC_MAIN";
// #define ALPHA 0.70

static SemaphoreHandle_t imu_data_mutex;
static telemetry_imu_payload_t latest_imu_payload;

void fc_task(void *args) { //semnatura unui task freeRTOS trebuie sa contine un param de tip void*
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

    const TickType_t xFreq = pdMS_TO_TICKS(1000); // 1ms -> 1kHz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    // uint64_t last_time = esp_timer_get_time();


    imu_raw_data_t imu_raw_data;
    imu_physical_data_t imu_physical_data;

    float roll_angle = 0.0f, pitch_angle = 0.0f;
    // float roll_angle_comp = 0.0f, pitch_angle_comp = 0.0f;
    for (;;) {
        error = imu_read_raw_data(&imu_raw_data);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "IMU read failed: %s", esp_err_to_name(error));
            vTaskDelay(pdMS_TO_TICKS(10)); // wait a bit before retrying
            continue;
        }
        imu_convert_raw_to_physical(&imu_raw_data, &imu_physical_data);

        
        // uint64_t current_time = esp_timer_get_time();
        // float dt = (current_time - last_time) / 1000000.0f; // convert to seconds
        // last_time = current_time;
        
        imu_compute_roll_pitch(imu_physical_data.acc_x_g, imu_physical_data.acc_y_g, imu_physical_data.acc_z_g, &roll_angle, &pitch_angle);
        
        if (imu_data_mutex != NULL && xSemaphoreTake(imu_data_mutex, portMAX_DELAY) == pdTRUE) {
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

        // ESP_LOGI(TAG, "IMU Data - Acc: (%.4f, %.4f, %.4f), Gyro: (%.4f, %.4f, %.4f), Temp: %.4f",
        // imu_physical_data.acc_x_g, imu_physical_data.acc_y_g, imu_physical_data.acc_z_g,
        // imu_physical_data.gyro_x_dps, imu_physical_data.gyro_y_dps, imu_physical_data.gyro_z_dps,
        // imu_physical_data.temp_C
        // );
        // ESP_LOGI(TAG, "Unghiuri - Roll: %.2f | Pitch: %.2f", roll_angle, pitch_angle);
        // ESP_LOGI(TAG, "Unghiuri calibrate - Roll: %.2f | Pitch: %.2f", roll_angle - roll_offset, pitch_angle - pitch_offset);
                 
        // 2. Calculează PID
        // 3. Scrie comanda PWM către motoarele coreless

        // Așteaptă exact până la următoarea milisecundă
        vTaskDelayUntil(&xLastWakeTime, xFreq);
    }
}

void telemetry_task(void *args) {
    //init uart...
    ESP_LOGI(TAG, "Telemtry transmission task started on core %d", xPortGetCoreID());

    telemetry_message_t rx_message;
    for (;;) {
        telemetry_imu_payload_t imu_payload = {0};

        if (imu_data_mutex != NULL && xSemaphoreTake(imu_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            imu_payload = latest_imu_payload;
            xSemaphoreGive(imu_data_mutex);
        }

        esp_err_t send_error = uart_telemetry_send_message(
            TELEMETRY_MSG_IMU,
            &imu_payload,
            sizeof(imu_payload)
        );
        if (send_error != ESP_OK) {
            ESP_LOGE(TAG, "IMU telemetry send failed: %s", esp_err_to_name(send_error));
        }

        if (uart_telemetry_read_message(&rx_message)) {
            if ((telemetry_msg_type_t)rx_message.header.msg_type == TELEMETRY_MSG_CONTROL) {
                const telemetry_control_payload_t *control = (const telemetry_control_payload_t *)rx_message.payload;
                ESP_LOGI(TAG, "CTRL seq=%u roll=%.2f pitch=%.2f yaw=%.2f thr=%.2f armed=%u mode=%u",
                         rx_message.header.seq,
                         control->roll_setpoint, control->pitch_setpoint, control->yaw_setpoint,
                         control->throttle, control->armed, control->flight_mode);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

}

void app_main(void)
{
    ESP_LOGI(TAG, "System starting up...");
    
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
    esp_err_t error;
    imu_data_mutex = xSemaphoreCreateMutex();
    if (imu_data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create IMU data mutex");
        return;
    }

    telemetry_uart_config_t uart_config = {
        .uart_port  = UART_NUM_1,
        .tx_pin     = GPIO_NUM_6, 
        .rx_pin     = GPIO_NUM_7, 
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
