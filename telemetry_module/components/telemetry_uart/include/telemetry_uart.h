#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "hal/uart_types.h"
#include "hal/gpio_types.h"
#include "esp_err.h"

#define PACKET_START_BYTE 0xAA
#define PACKET_END_BYTE 0xBB

typedef struct {
    uart_port_t uart_port;      // Portul UART utilizat pentru comunicație
    gpio_num_t tx_pin;          // Pinul TX pentru UART
    gpio_num_t rx_pin;          // Pinul RX pentru UART
    uint32_t baud_rate;         // Viteza de transmisie UART
} telemetry_uart_config_t;

typedef struct {
    uint8_t start_byte;
    float roll_angle;
    float pitch_angle;
    //mai putem adauga si alte date de zbor aici
    uint8_t end_byte;
}__attribute__((packed)) telemetry_packet_t;

esp_err_t uart_telemetry_init(const telemetry_uart_config_t *config);
bool uart_telemetry_read_packet(telemetry_packet_t *out_packet);
void uart_telemetry_send_packet(const telemetry_packet_t *packet);