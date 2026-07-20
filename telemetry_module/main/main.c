#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "telemetry_uart.h"
#include "web_page.h"

static const char *TAG = "Telemetry Module";

void telemetry_task(void *pvParameters) {
    ESP_LOGI("Telemetry Task", "Telemetry task started");

    telemetry_message_t rx_message;
    telemetry_control_payload_t control_frame = {
        .roll_setpoint = 0.0f,
        .pitch_setpoint = 0.0f,
        .yaw_setpoint = 0.0f,
        .throttle = 0.0f,
        .armed = 0,
        .flight_mode = 0,
    };

    for (;;) {

        esp_err_t send_error = uart_telemetry_send_message(TELEMETRY_MSG_CONTROL, &control_frame, sizeof(control_frame));
        if (send_error != ESP_OK) {
            ESP_LOGW(TAG, "Control frame send failed: %s", esp_err_to_name(send_error));
        }
        else {
            ESP_LOGI(TAG, "Control frame sent successfully");
        }

        if (uart_telemetry_read_message(&rx_message)) {
            web_page_store_message(&rx_message);
            switch ((telemetry_msg_type_t)rx_message.header.msg_type) {
                case TELEMETRY_MSG_IMU: {
                    const telemetry_imu_payload_t *imu = (const telemetry_imu_payload_t *)rx_message.payload;
                    ESP_LOGI(TAG, "IMU seq=%u acc=(%.2f %.2f %.2f) gyro=(%.2f %.2f %.2f) temp=%.2f",
                             rx_message.header.seq,
                             imu->acc_x, imu->acc_y, imu->acc_z,
                             imu->gyro_x, imu->gyro_y, imu->gyro_z,
                             imu->temperature);
                    break;
                }
                case TELEMETRY_MSG_CONTROL: {
                    const telemetry_control_payload_t *control = (const telemetry_control_payload_t *)rx_message.payload;
                    ESP_LOGI(TAG, "CTRL seq=%u roll=%.2f pitch=%.2f yaw=%.2f thr=%.2f armed=%u mode=%u",
                             rx_message.header.seq,
                             control->roll_setpoint, control->pitch_setpoint, control->yaw_setpoint,
                             control->throttle, control->armed, control->flight_mode);
                    break;
                }
                case TELEMETRY_MSG_BATTERY: {
                    const telemetry_battery_payload_t *battery = (const telemetry_battery_payload_t *)rx_message.payload;
                    ESP_LOGI(TAG, "BAT seq=%u voltage=%.2f current=%.2f percent=%u",
                             rx_message.header.seq,
                             battery->battery_voltage, battery->battery_current, battery->battery_percent);
                    break;
                }
                default:
                    ESP_LOGW(TAG, "Unknown message type %u seq=%u len=%u",
                             rx_message.header.msg_type,
                             rx_message.header.seq,
                             rx_message.header.payload_len);
                    break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Telemtry module starting up...");
    esp_err_t error;

    ESP_ERROR_CHECK(web_page_init());

    telemetry_uart_config_t uart_config = {
        .uart_port = UART_NUM_1,  // Alegem UART1 pentru comunicație
        .tx_pin = GPIO_NUM_9,    // Pinul TX (transmitere)
        .rx_pin = GPIO_NUM_8,    // Pinul RX (recepție)
        .baud_rate = 460800       // Viteza de transmisie
    };

    error = uart_telemetry_init(&uart_config);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize UART telemetry! Error returned: %s", esp_err_to_name(error));
        abort();  
    }
    else {
        ESP_LOGI(TAG, "UART telemetry initialized successfully.");
    }

    xTaskCreatePinnedToCore(telemetry_task, "TELEMETRY_TASK", 4096, NULL, 5, NULL, 0);

    error = web_page_start_task(0);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start web page task: %s", esp_err_to_name(error));
        abort();
    }
}   
