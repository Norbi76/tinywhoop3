#pragma once

#include <stdint.h>
#include "esp_err.h"

// motor_driver.h - the last stage of the control chain: four normalised thrusts -> four PWM duties.
//
// WHAT THIS COMPONENT IS FOR
//   flight_control's mixer produces four numbers in 0.0..1.0; this component turns them into duty
//   cycles on four MOSFET gates using the ESP32-S3's LEDC peripheral. It is deliberately thin -
//   no mixing, no arming logic, no rate limiting, no state machine. It clamps, maps, writes, and
//   remembers what it wrote. Everything policy-shaped stays in flight_control.
//
// HOW IT DOES IT
//   One LEDC timer (24 kHz, 11-bit) shared by four channels, so all four gates are driven from
//   the same time base. A write is ledc_set_duty() + ledc_update_duty(), BOTH error-checked - a
//   silent failure of the second one means a motor keeps its previous command, which during a
//   disarm means it keeps spinning. The multi-motor calls deliberately continue past a failure
//   and report the first error, because a partial stop is worse than a reported one.
//
// LEDC-based driver for four coreless brushed motors driven through MOSFET gates.
//
// MOTOR ORDERING - the mixer in flight_control depends on this and it MUST be verified
// on the bench before the props go on:
//
//            FRONT
//       M0 .......... M1        M0 = front-left,  spins CW 
//        :            :         M1 = front-right, spins CCW
//        :     +      :         M2 = rear-left,   spins CCW
//        :            :         M3 = rear-right,  spins CW
//       M2 .......... M3
//            REAR
//
// The CW/CCW assignment above determines the sign of the yaw column in the mixer. Get it
// wrong and the drone will spin up in yaw instead of holding heading.
#define MOTOR_COUNT 4

#define MOTOR_FRONT_LEFT  0
#define MOTOR_FRONT_RIGHT 1
#define MOTOR_REAR_LEFT   2
#define MOTOR_REAR_RIGHT  3

// Initialises the LEDC timer and the four output channels, and leaves all motors stopped.
// @return ESP_OK on success, or the first LEDC error encountered.
esp_err_t motor_driver_init(void);

// Sets one motor's thrust as a normalised command.
// Applies the thrust->duty map. There is no battery-voltage compensation - the airframe has no
// pack ADC, so thrust maps straight to duty and a sagging pack simply produces less thrust.
// @param motor_index 0..3, see the MOTOR_* defines above.
// @param thrust 0.0 (stopped) .. 1.0 (full). Values outside the range are clamped.
// @return ESP_OK on success, ESP_ERR_INVALID_ARG for a bad index, ESP_ERR_INVALID_STATE if
//         the driver is not initialised, or the underlying LEDC error.
esp_err_t motor_set_thrust(int motor_index, float thrust);

// Sets all four motors in one call. Attempts every channel even if one fails, so a single bad
// channel cannot leave the other three running.
// @param thrust Array of MOTOR_COUNT normalised thrust values.
// @return ESP_OK if every channel succeeded, otherwise the first error encountered.
esp_err_t motor_set_thrust_all(const float thrust[MOTOR_COUNT]);

// Writes a raw LEDC duty value, bypassing the thrust curve.
// Bench testing only - this is how you find HOVER_THROTTLE and check motor wiring.
// @param motor_index 0..3.
// @param duty Raw duty, 0 .. (2^MOTOR_PWM_RESOLUTION_BITS - 1). Clamped to the maximum.
// @return ESP_OK on success, or an error code on failure.
esp_err_t motor_set_raw_duty(int motor_index, uint32_t duty);

// Forces all four motors to zero duty immediately.
// Called on disarm, on kill, and on the link watchdog firing.
// Attempts all four channels regardless of individual failures.
// @return ESP_OK if every channel stopped, otherwise the first error encountered.
esp_err_t motor_all_stop(void);

// Returns the duty most recently written to a motor, for telemetry and bench inspection.
// @param motor_index 0..3.
// @return The last raw duty written, or 0 for a bad index.
uint32_t motor_get_last_duty(int motor_index);
