#include "telemetry_uart.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>
#include "esp_err.h"

#define UART_BUF_SIZE (1024 * 2)

static uart_port_t active_uart_num; 

esp_err_t uart_telemetry_init(const telemetry_uart_config_t *config) {
    esp_err_t error;
    
    active_uart_num = config->uart_port;
    const uart_config_t uart_cfg = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    error = uart_param_config(active_uart_num, &uart_cfg);
    if (error != ESP_OK) {
        return error;
    }

    error = uart_set_pin(active_uart_num, config->tx_pin, config->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (error != ESP_OK) {
        return error;
    }

    error = uart_driver_install(active_uart_num, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0);
    if (error != ESP_OK) {
        return error;
    }

    return ESP_OK;
}

bool uart_telemetry_read_packet(telemetry_packet_t *out_packet) {
    uint8_t rx_buffer[sizeof(telemetry_packet_t)];
    
    // non-blocking reading (timeout foarte mic, 2 milisecunde)
    int rx_bytes = uart_read_bytes(active_uart_num, rx_buffer, sizeof(telemetry_packet_t), pdMS_TO_TICKS(2));

    if (rx_bytes == sizeof(telemetry_packet_t)) {
        telemetry_packet_t *temp_packet = (telemetry_packet_t *)rx_buffer;

        if (temp_packet->start_byte == PACKET_START_BYTE && temp_packet->end_byte == PACKET_END_BYTE) {
            memcpy(out_packet, temp_packet, sizeof(telemetry_packet_t));
            return true;
        } else {
            uart_flush_input(active_uart_num);
        }
    }
    
    return false;
}

void uart_telemetry_send_packet(const telemetry_packet_t *packet) {
    uart_write_bytes(active_uart_num, (const char *)packet, sizeof(telemetry_packet_t));
}