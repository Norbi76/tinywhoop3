// telemetry_uart.h - the wire contract between the flight controller and the telemetry module.
//
// WHAT THIS FILE IS FOR
//   This header IS the protocol. Everything the two boards agree on lives here: the frame layout,
//   the message type numbers, the packed payload structs, the flag bits and the two PID loop-id
//   enums. Both firmware projects compile against this one copy of the file (pulled in via
//   EXTRA_COMPONENT_DIRS, not vendored), so a change here changes both sides at once - which is
//   exactly why it is safe, and exactly why it must never be edited with only one side in mind.
//
// HOW IT KEEPS THE TWO SIDES HONEST
//   The payload structs are __attribute__((packed)) and are memcpy'd straight into and out of the
//   frame buffer - there is no serialisation step that could paper over a layout difference. The
//   _Static_assert block at the bottom of the file is the enforcement mechanism: it fails the
//   BUILD when a payload grows past the frame size, or when one of the PID payloads changes size
//   at all (the Python ground station unpacks those with hardcoded struct formats).
//
// See README.md in this directory for the frame diagram, the message table, and why there are two
// different loop-id enums for the same eight PID instances.

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

    // --- PID tuning link (flight controller <-> Python ground station) -------
    // These ride the same UART as everything else, but the telemetry module does not interpret
    // them: it forwards them verbatim between the UART and the UDP ground-station link.
    TELEMETRY_MSG_PID_DEBUG = 20,   // FC -> GS, high rate
    TELEMETRY_MSG_PID_GAINS = 21,   // both directions
    TELEMETRY_MSG_PID_SELECT = 22,  // GS -> FC
    TELEMETRY_MSG_PID_INJECT = 23,  // GS -> FC
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
// [payload: 0-64 bytes, length given by payload_len]                0-64 bytes
// [tail: crc16(2) | end_byte(1)]                                       3 bytes
// The payload bound is TELEMETRY_MAX_PAYLOAD_SIZE (64). Note the PID payloads are additionally
// asserted against the tighter TELEMETRY_PID_PAYLOAD_CEILING (48) - see the assert block below.
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

// ---------------------------------------------------------------------------
// PID TUNING LINK
//
// A second, independent addressing of the same 8 PID instances, used by the Python ground
// station. It exists alongside telemetry_loop_id_t rather than replacing it because the two
// orderings differ and the web dashboard's loop ids are baked into index.html - renumbering
// telemetry_loop_id_t would silently retune the wrong axis from the browser.
//
// pid_registry_bind() on the flight controller is what reconciles the two: each PID instance is
// bound to its pid_loop_id_t while still living at its telemetry_loop_id_t index.
// ---------------------------------------------------------------------------

// Ground-station loop ids. Ordered outer-to-inner, which is the order the tuning UI enumerates.
typedef enum {
    PID_LOOP_ALT = 0,         // m/s in    -> throttle out,  50 Hz
    PID_LOOP_VEL_X = 1,       // m/s in    -> deg out,       50 Hz
    PID_LOOP_VEL_Y = 2,       // m/s in    -> deg out,       50 Hz
    PID_LOOP_ANG_ROLL = 3,    // deg in    -> deg/s out,    250 Hz
    PID_LOOP_ANG_PITCH = 4,   // deg in    -> deg/s out,    250 Hz
    PID_LOOP_RATE_ROLL = 5,   // deg/s in  -> cmd out,     1000 Hz
    PID_LOOP_RATE_PITCH = 6,  // deg/s in  -> cmd out,     1000 Hz
    PID_LOOP_RATE_YAW = 7,    // deg/s in  -> cmd out,     1000 Hz
    PID_LOOP_COUNT = 8,
} pid_loop_id_t;

// Wildcard loop id for telemetry_pid_select_payload_t: stream every loop at once.
// 0xFF rather than PID_LOOP_COUNT so that a future ninth loop cannot collide with it.
#define PID_LOOP_ALL 0xFF

// Bit flags for telemetry_pid_debug_payload_t.flags.
#define PID_FLAG_I_CLAMPED  (1u << 0) // integrator hit its limit or was frozen by anti-windup
#define PID_FLAG_OUT_SAT    (1u << 1) // output was clipped by the saturation limits
#define PID_FLAG_MEAS_STALE (1u << 2) // measurement is older than one loop period

// Every PID payload must also fit inside this tighter ceiling. TELEMETRY_MAX_PAYLOAD_SIZE was
// raised to 64 for the status frame, but the ground station's struct formats are hardcoded
// against 48 - keeping the assert at 48 means the Python side cannot silently fall behind.
#define TELEMETRY_PID_PAYLOAD_CEILING 48

// One sample of a single PID loop's internals (flight controller -> ground station).
// Emitted from inside the control task at up to the loop's own rate, so it is deliberately the
// smallest thing that still lets the whole loop be reconstructed offline.
//
// The error is NOT transmitted: it is exactly setpoint - measurement, and at 500 Hz those four
// bytes are a fifth of the link budget for something the receiver can compute itself.
typedef struct {
    uint32_t t_us;        // esp_timer_get_time() truncated to 32 bit - wraps every ~71 minutes
    uint8_t loop_id;      // pid_loop_id_t
    uint8_t flags;        // PID_FLAG_*
    float setpoint;       // in the loop's setpoint units, injection offset included
    float measurement;
    float p_term;
    float i_term;         // the ki-SCALED integrator, not the raw error integral
    float d_term;
    float output;         // post-clamp, i.e. what the next stage actually received
} __attribute__((packed)) telemetry_pid_debug_payload_t;

// Full gain set for one loop. Travels in BOTH directions: the ground station sends it to set
// gains, and the flight controller sends the same struct back to report them.
//
// A message whose six float fields are all exactly zero is a READ REQUEST, not a set. An
// all-zero gain set would disable the loop entirely and is never something anyone means, so the
// overload is safe and saves a message type.
typedef struct {
    uint8_t loop_id;      // pid_loop_id_t
    float kp;
    float ki;
    float kd;
    float i_limit;        // symmetric integrator clamp
    float out_limit;      // symmetric output clamp, applied as out_min = -x, out_max = +x
    float d_cutoff_hz;    // derivative low-pass corner
} __attribute__((packed)) telemetry_pid_gains_payload_t;

// Subscribes the debug stream to one loop (ground station -> flight controller).
// Only one selection is active at a time: the UART cannot carry eight loops at full rate.
typedef struct {
    uint8_t loop_id;      // pid_loop_id_t, or PID_LOOP_ALL to stream every loop
    uint8_t divider;      // emit 1 sample per N loop ticks; 0 is treated as 1
    uint16_t reserved;
} __attribute__((packed)) telemetry_pid_select_payload_t;

// Test-signal injection (ground station -> flight controller).
// Adds a perturbation to one loop's setpoint so its step response can be measured in flight,
// which is what the overshoot and settling-time readouts on the ground station are computed from.
typedef struct {
    uint8_t loop_id;      // pid_loop_id_t
    uint8_t mode;         // 0 off, 1 step, 2 doublet, 3 square
    float amplitude;      // in the loop's setpoint units
    float period_s;
} __attribute__((packed)) telemetry_pid_inject_payload_t;

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

// The PID payloads get EXACT size asserts on top of the fits-in-a-frame check, because the
// ground station unpacks them with hardcoded struct formats ("<IBB6f" and friends). A padding
// byte introduced by a compiler or an added field would still fit the frame and would still
// build - it would just shift every float in the plot by one byte and produce garbage that
// looks like noise rather than like a bug. Fail the build instead.
_Static_assert(sizeof(telemetry_pid_debug_payload_t) == 30,
               "PID debug payload must be exactly 30 bytes - the ground station unpacks '<IBB6f'");
_Static_assert(sizeof(telemetry_pid_gains_payload_t) == 25,
               "PID gains payload must be exactly 25 bytes - the ground station unpacks '<B6f'");
_Static_assert(sizeof(telemetry_pid_select_payload_t) == 4,
               "PID select payload must be exactly 4 bytes - the ground station unpacks '<BBH'");
_Static_assert(sizeof(telemetry_pid_inject_payload_t) == 10,
               "PID inject payload must be exactly 10 bytes - the ground station unpacks '<BBff'");

_Static_assert(sizeof(telemetry_pid_debug_payload_t) <= TELEMETRY_PID_PAYLOAD_CEILING,
               "PID debug payload does not fit in a telemetry frame");
_Static_assert(sizeof(telemetry_pid_gains_payload_t) <= TELEMETRY_PID_PAYLOAD_CEILING,
               "PID gains payload does not fit in a telemetry frame");
_Static_assert(sizeof(telemetry_pid_select_payload_t) <= TELEMETRY_PID_PAYLOAD_CEILING,
               "PID select payload does not fit in a telemetry frame");
_Static_assert(sizeof(telemetry_pid_inject_payload_t) <= TELEMETRY_PID_PAYLOAD_CEILING,
               "PID inject payload does not fit in a telemetry frame");

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