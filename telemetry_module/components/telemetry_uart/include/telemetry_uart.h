#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "hal/uart_types.h"
#include "hal/gpio_types.h"
#include "esp_err.h"

#define TELEMETRY_START_BYTE 0xAA
#define TELEMETRY_END_BYTE 0xBB
#define TELEMETRY_PROTOCOL_VERSION 1
#define TELEMETRY_MAX_PAYLOAD_SIZE 48

typedef enum {
    TELEMETRY_MSG_IMU = 1,
    TELEMETRY_MSG_BATTERY = 2,
    TELEMETRY_MSG_STATUS = 3,
    TELEMETRY_MSG_CONTROL = 10,
    TELEMETRY_MSG_ARMING = 11,
    TELEMETRY_MSG_MODE = 12,
} telemetry_msg_type_t;

typedef struct {
    uart_port_t uart_port;      // Portul UART utilizat pentru comunicație
    gpio_num_t tx_pin;          // Pinul TX pentru UART
    gpio_num_t rx_pin;          // Pinul RX pentru UART
    uint32_t baud_rate;         // Viteza de transmisie UART
} telemetry_uart_config_t;

typedef struct {
    uint8_t start_byte;
    uint8_t version;
    uint8_t msg_type;
    uint8_t payload_len;
    uint16_t seq;
} __attribute__((packed)) telemetry_frame_header_t;

typedef struct {
    uint16_t crc16;
    uint8_t end_byte;
} __attribute__((packed)) telemetry_frame_tail_t;

typedef struct {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float roll;
    float pitch;
    float temperature;
} __attribute__((packed)) telemetry_imu_payload_t;

typedef struct {
    float battery_voltage;
    float battery_current;
    uint8_t battery_percent;
} __attribute__((packed)) telemetry_battery_payload_t;

typedef struct {
    float roll_setpoint;
    float pitch_setpoint;
    float yaw_setpoint;
    float throttle;
    uint8_t armed;
    uint8_t flight_mode;
} __attribute__((packed)) telemetry_control_payload_t;

typedef struct {
    telemetry_frame_header_t header;
    uint8_t payload[TELEMETRY_MAX_PAYLOAD_SIZE];
    telemetry_frame_tail_t tail;
} __attribute__((packed)) telemetry_message_t;

esp_err_t uart_telemetry_init(const telemetry_uart_config_t *config);
bool uart_telemetry_read_message(telemetry_message_t *out_message);
esp_err_t uart_telemetry_send_message(telemetry_msg_type_t msg_type, const void *payload, uint8_t payload_len);
uint16_t telemetry_calculate_crc16(const uint8_t *data, size_t length);