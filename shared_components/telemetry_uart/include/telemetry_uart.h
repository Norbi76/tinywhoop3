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
// Raised from 48 to 64 so telemetry_status_payload_t (53 bytes) fits in one frame.
#define TELEMETRY_MAX_PAYLOAD_SIZE 64

typedef enum {
    TELEMETRY_MSG_IMU = 1,
    TELEMETRY_MSG_BATTERY = 2,
    TELEMETRY_MSG_STATUS = 3,
    TELEMETRY_MSG_CONTROL = 10,
    TELEMETRY_MSG_ARMING = 11,
    TELEMETRY_MSG_MODE = 12,
    TELEMETRY_MSG_GAINS = 13,
} telemetry_msg_type_t;

// Flight modes, carried in telemetry_control_payload_t.flight_mode and echoed back in the status frame.
// The outer loops (altitude / velocity) only engage in the modes that ask for them AND when the
// matching nav validity flag is set - see flight_control_update().
typedef enum {
    TELEMETRY_MODE_ANGLE = 0,    // Manual: stick -> roll/pitch angle, throttle passed straight through.
    TELEMETRY_MODE_ALT_HOLD = 1, // Altitude held by the ToF-driven outer loop, roll/pitch still manual.
    TELEMETRY_MODE_POS_HOLD = 2, // Altitude held AND body velocity driven to zero by optical flow.
} telemetry_flight_mode_t;

// Bit flags for telemetry_control_payload_t.flags (telemetry module -> flight controller).
#define TELEMETRY_CTRL_FLAG_KILL     (1u << 0) // Latching emergency cut. Overrides everything, needs a reboot-free re-arm.
#define TELEMETRY_CTRL_FLAG_HOLD     (1u << 1) // Dashboard "hold" toggle: request the position/altitude hold mode.
#define TELEMETRY_CTRL_FLAG_CAPTURE  (1u << 2) // Informational only: a photo was requested on this frame.

// Bit flags for telemetry_status_payload_t.flags (flight controller -> telemetry module).
#define TELEMETRY_STATUS_FLAG_LINK_OK        (1u << 0) // Control link watchdog is satisfied.
#define TELEMETRY_STATUS_FLAG_ATTITUDE_INIT  (1u << 1) // Complementary filter has settled, safe to arm.
#define TELEMETRY_STATUS_FLAG_ALT_VALID      (1u << 2) // nav_estimator altitude is trustworthy.
#define TELEMETRY_STATUS_FLAG_VEL_VALID      (1u << 3) // nav_estimator body velocity is trustworthy.
#define TELEMETRY_STATUS_FLAG_KILLED         (1u << 4) // Kill latch is engaged.

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

// Represents the payload of a control telemetry message (telemetry module -> flight controller).
// Sent at a fixed 50 Hz whether or not the pilot is touching anything, because the flight
// controller's link watchdog disarms after 300 ms without a valid frame.
typedef struct {
    float roll_setpoint;   // desired roll angle, degrees, + = right wing down
    float pitch_setpoint;  // desired pitch angle, degrees, + = nose up
    float yaw_setpoint;    // desired yaw RATE, degrees/second, + = nose right (yaw has no angle loop)
    float throttle;        // 0.0 .. 1.0
    uint8_t armed;         // 1 = the pilot is requesting arm; the FC still applies its own arming gate
    uint8_t flight_mode;   // telemetry_flight_mode_t
    uint8_t flags;         // TELEMETRY_CTRL_FLAG_*
} __attribute__((packed)) telemetry_control_payload_t;

// Represents the payload of a status telemetry message (flight controller -> telemetry module).
// This is what the dashboard reads back; it is the FC's view of the world, not the pilot's request.
typedef struct {
    float roll;             // degrees, from the complementary filter
    float pitch;            // degrees
    float yaw;              // degrees, gyro integration only - drifts, no magnetometer
    float altitude;         // metres above the surface under the drone (tilt-compensated ToF)
    float climb_rate;       // metres/second, + = climbing
    float velocity_x;       // body-frame forward velocity, m/s, from optical flow
    float velocity_y;       // body-frame right velocity, m/s
    float battery_voltage;  // volts
    float motor[4];         // final normalised motor commands 0..1, in mixer order (see motor_driver.h)
    uint16_t loop_hz;       // measured rate inner loop frequency, for verifying the 1 kHz loop on the bench
    uint8_t armed;          // 1 = motors are live
    uint8_t flight_mode;    // telemetry_flight_mode_t actually in effect (may differ from requested)
    uint8_t flags;          // TELEMETRY_STATUS_FLAG_*
} __attribute__((packed)) telemetry_status_payload_t;

// Identifies which of the 8 PID instances a gains message addresses.
// The order here must match pid_loop_id_t on the flight controller side.
typedef enum {
    TELEMETRY_LOOP_RATE_ROLL = 0,
    TELEMETRY_LOOP_RATE_PITCH = 1,
    TELEMETRY_LOOP_RATE_YAW = 2,
    TELEMETRY_LOOP_ANGLE_ROLL = 3,
    TELEMETRY_LOOP_ANGLE_PITCH = 4,
    TELEMETRY_LOOP_VEL_X = 5,
    TELEMETRY_LOOP_VEL_Y = 6,
    TELEMETRY_LOOP_ALTITUDE = 7,
    TELEMETRY_LOOP_COUNT = 8,
} telemetry_loop_id_t;

// Represents the payload of a gain-update message (telemetry module -> flight controller).
// One loop per message, so the dashboard can tune a single axis without disturbing the others.
typedef struct {
    uint8_t loop_id;  // telemetry_loop_id_t
    float kp;
    float ki;
    float kd;
} __attribute__((packed)) telemetry_gains_payload_t;

// Compile-time guard: every payload must fit inside one frame.
// Without these, adding a field to a payload struct would silently overflow
// telemetry_message_t.payload and corrupt the frame tail at runtime, which presents as
// intermittent CRC failures rather than as an obvious bug.
_Static_assert(sizeof(telemetry_imu_payload_t) <= TELEMETRY_MAX_PAYLOAD_SIZE,
               "IMU payload does not fit in a telemetry frame");
_Static_assert(sizeof(telemetry_battery_payload_t) <= TELEMETRY_MAX_PAYLOAD_SIZE,
               "Battery payload does not fit in a telemetry frame");
_Static_assert(sizeof(telemetry_control_payload_t) <= TELEMETRY_MAX_PAYLOAD_SIZE,
               "Control payload does not fit in a telemetry frame");
_Static_assert(sizeof(telemetry_status_payload_t) <= TELEMETRY_MAX_PAYLOAD_SIZE,
               "Status payload does not fit in a telemetry frame");
_Static_assert(sizeof(telemetry_gains_payload_t) <= TELEMETRY_MAX_PAYLOAD_SIZE,
               "Gains payload does not fit in a telemetry frame");

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

// Uart read message function with byte-level resynchronisation.
// Behaves like uart_telemetry_read_message(), except that it never calls uart_flush_input():
// on a bad start byte, bad length, bad end byte or CRC mismatch it discards only the bytes it has
// already consumed and keeps scanning the stream for the next start byte.
//
// Prefer this over uart_telemetry_read_message() on a link that carries steady periodic traffic.
// Flushing the whole RX buffer on a single corrupted byte also throws away the good frames queued
// behind it, which at 50 Hz is enough sustained loss to trip the flight controller's 300 ms
// link watchdog. This function loses one frame instead of all of them.
//
// @param out_message Pointer to a telemetry_message_t structure where the read message will be stored.
// @param timeout_ms Total time budget for finding and reading one complete frame.
// @return Returns true if a valid message was read, false on timeout or if no valid frame was found.
bool uart_telemetry_read_message_resync(telemetry_message_t *out_message, uint32_t timeout_ms);

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