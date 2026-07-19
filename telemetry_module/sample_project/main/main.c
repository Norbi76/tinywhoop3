#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "telemetry_uart.h"

static const char *TAG = "Telemetry Module";

void telemetry_task(void *pvParameters) {
    ESP_LOGI("Telemetry Task", "Telemetry task started");


    telemetry_packet_t rx_packet;
    for (;;) {
        
        if (uart_telemetry_read_packet(&rx_packet)) {
            ESP_LOGI(TAG, "Received telemetry packet: Roll: %.2f, Pitch: %.2f", rx_packet.roll_angle, rx_packet.pitch_angle);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Telemtry module starting up...");
    esp_err_t error;

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

    xTaskCreatePinnedToCore(
        telemetry_task,       // Funcția task-ului
        "TELEMETRY_TASK",     // Nume intern (pentru debug)
        4096,                 // Memorie alocată (Stivă)
        NULL,                 // Parametri
        5,                    // Prioritate (5 e o prioritate medie-mare)
        NULL,                 // Handle
        0                     // Alocat pe Core 1
    );
}   
