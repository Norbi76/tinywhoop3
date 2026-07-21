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

// Represents the configuration for the UART interface(UART port, TX and RX pins, baud rate).
typedef struct {
    uart_port_t uart_port;      
    gpio_num_t tx_pin;          
    gpio_num_t rx_pin;          
    uint32_t baud_rate;
} telemetry_uart_config_t;

// Represents the header of a telemetry message frame.
typedef struct {
    uint8_t start_byte;
    uint8_t msg_type;
    uint8_t payload_len;
    uint16_t seq;
} __attribute__((packed)) telemetry_frame_header_t;

// Represents the tail of a telemetry message frame.
typedef struct {
    uint16_t crc16;
    uint8_t end_byte;
} __attribute__((packed)) telemetry_frame_tail_t;

//Represents a complete telemetry message frame, including the header, payload, and tail.
// [header: start_byte(1) | msg_type(1) | payload_len(1) | seq(2)]      5 bytes
// [payload: 0-48 bytes, length given by payload_len]                0-48 bytes
// [tail: crc16(2) | end_byte(1)]                                       3 bytes
typedef struct {
    telemetry_frame_header_t header;
    uint8_t payload[TELEMETRY_MAX_PAYLOAD_SIZE];
    telemetry_frame_tail_t tail;
} __attribute__((packed)) telemetry_message_t;

// Represents the payload of an IMU telemetry message.
// Includes physical data and calculated roll and pitch angles.
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

// Represents the payload of a battery telemetry message.
typedef struct {
    float battery_voltage;
    float battery_current;
    uint8_t battery_percent;
} __attribute__((packed)) telemetry_battery_payload_t;

// Represents the payload of a control telemetry message.
typedef struct {
    float roll_setpoint;
    float pitch_setpoint;
    float yaw_setpoint;
    float throttle;
    uint8_t armed;
    uint8_t flight_mode;
} __attribute__((packed)) telemetry_control_payload_t;

// Uart initialization function.
// Sets up the UART interface with the specified configuration parameter.
//@param config Pointer to a telemetry_uart_config_t structure containing the UART configuration parameters.
//@return Returns ESP_OK on success, or an error code on failure.
esp_err_t uart_telemetry_init(const telemetry_uart_config_t *config);

// Uart read message function.
// Reads the header, payload and tail of a telemetry message from the UART interface.
// Only after all checks for the header, payload and tail are passed they get copied into the caller's out_meessage structure.
// @param out_message Pointer to a telemetry_message_t structure where the read message will be stored
// @return Returns true if a valid message was read, false otherwise.
bool uart_telemetry_read_message(telemetry_message_t *out_message);

// Uart send message function.
// Sends a telemetry message over the UART interface, including the header, payload, and tail.
// @param msg_type The type of the telemetry message to be sent (e.g., TELEMETRY_MSG_IMU, TELEMETRY_MSG_BATTERY, etc.)
// @param payload Pointer to the payload data to be sent. The payload should match the expected structure for the specified message type.
// @param payload_len The length of the payload data in bytes. Must not exceed TELEMETRY_MAX_PAYLOAD_SIZE.
// @return Returns ESP_OK on success, or an error code on failure. 
esp_err_t uart_telemetry_send_message(telemetry_msg_type_t msg_type, const void *payload, uint8_t payload_len);

// Calculates the CRC16 checksum for the given data buffer.
// @param data Pointer to the data buffer for which the CRC16 checksum will be calculated.
// @param length The length of the data buffer in bytes.
// @return Returns the calculated CRC16 checksum as a 16-bit unsigned integer.
uint16_t telemetry_calculate_crc16(const uint8_t *data, size_t length);