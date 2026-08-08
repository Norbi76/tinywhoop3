#include "motor_driver.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "MOTOR";

// ---------------------------------------------------------------------------
// TODO(pins): CONFIRM AGAINST YOUR WIRING before the first powered test.
// These are placeholders. On the ESP32-S3 any GPIO can drive LEDC, but avoid the strapping
// pins (0, 3, 45, 46) and the USB-JTAG pins (19, 20) for motor gates - a strapping pin held
// high or low at boot by a gate pull-down will change the boot mode.
// GPIO 11 and 12 are already taken by the IMU I2C bus (see imu_driver.c).
// ---------------------------------------------------------------------------
#define MOTOR_FRONT_LEFT_GPIO  4
#define MOTOR_FRONT_RIGHT_GPIO 5
#define MOTOR_REAR_LEFT_GPIO   1
#define MOTOR_REAR_RIGHT_GPIO  2

// ---------------------------------------------------------------------------
// TODO(hardware): GATE PULL-DOWNS - VERIFY BEFORE FITTING PROPS.
// Between reset and motor_driver_init() completing, these GPIOs float. A floating MOSFET
// gate can drift above threshold and spin a motor while you are holding the drone.
// Each gate needs a physical pull-down resistor (~10k to GND) on the PCB. Confirm they are
// fitted and measure the gate voltage during a reboot with a scope before trusting this.
// Software cannot fix a floating gate - do not rely on the init sequence below for safety.
// ---------------------------------------------------------------------------

#define MOTOR_LEDC_TIMER      LEDC_TIMER_0
#define MOTOR_LEDC_MODE       LEDC_LOW_SPEED_MODE

// 24 kHz keeps the switching whine above the audible-annoyance band and is well inside what
// a small MOSFET can switch cleanly. At 11-bit resolution the LEDC hardware ceiling is
// 80 MHz / 2^11 = 39 kHz, so 24 kHz fits with margin.
#define MOTOR_PWM_FREQ_HZ            24000
#define MOTOR_PWM_RESOLUTION_BITS    LEDC_TIMER_11_BIT
#define MOTOR_PWM_MAX_DUTY           ((1u << 11) - 1u)   // 2047

static const int motor_gpio[MOTOR_COUNT] = {
    MOTOR_FRONT_LEFT_GPIO,
    MOTOR_FRONT_RIGHT_GPIO,
    MOTOR_REAR_LEFT_GPIO,
    MOTOR_REAR_RIGHT_GPIO,
};

static const ledc_channel_t motor_channel[MOTOR_COUNT] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3,
};

static bool driver_initialized;
static uint32_t last_duty[MOTOR_COUNT];

static float clampf(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// Writes a raw duty to a channel and latches it.
// Both ledc_set_duty() and ledc_update_duty() are checked - a silent failure here means a
// motor keeps its previous command, which during a disarm would mean it keeps spinning.
static esp_err_t motor_write_duty(int motor_index, uint32_t duty) {
    esp_err_t error = ledc_set_duty(MOTOR_LEDC_MODE, motor_channel[motor_index], duty);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "ledc_set_duty failed on motor %d: %s", motor_index, esp_err_to_name(error));
        return error;
    }

    error = ledc_update_duty(MOTOR_LEDC_MODE, motor_channel[motor_index]);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "ledc_update_duty failed on motor %d: %s", motor_index, esp_err_to_name(error));
        return error;
    }

    last_duty[motor_index] = duty;
    return ESP_OK;
}

esp_err_t motor_driver_init(void) {
    if (driver_initialized) {
        return ESP_OK;
    }

    esp_err_t error;

    const ledc_timer_config_t timer_config = {
        .speed_mode      = MOTOR_LEDC_MODE,
        .timer_num       = MOTOR_LEDC_TIMER,
        .duty_resolution = MOTOR_PWM_RESOLUTION_BITS,
        .freq_hz         = MOTOR_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };

    error = ledc_timer_config(&timer_config);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(error));
        return error;
    }

    for (int i = 0; i < MOTOR_COUNT; i++) {
        const ledc_channel_config_t channel_config = {
            .gpio_num   = motor_gpio[i],
            .speed_mode = MOTOR_LEDC_MODE,
            .channel    = motor_channel[i],
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = MOTOR_LEDC_TIMER,
            .duty       = 0,          // start stopped, before anything else can command thrust
            .hpoint     = 0,
        };

        error = ledc_channel_config(&channel_config);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "ledc_channel_config failed on motor %d (GPIO %d): %s",
                     i, motor_gpio[i], esp_err_to_name(error));
            return error;
        }

        last_duty[i] = 0;
    }

    driver_initialized = true;

    // Belt and braces: explicitly drive every channel to zero now that the driver is up.
    error = motor_all_stop();
    if (error != ESP_OK) {
        driver_initialized = false;
        return error;
    }

    ESP_LOGI(TAG, "Motor driver ready: %d channels, %d Hz, %d-bit (max duty %lu)",
             MOTOR_COUNT, MOTOR_PWM_FREQ_HZ, 11, (unsigned long)MOTOR_PWM_MAX_DUTY);

    return ESP_OK;
}

// Maps a normalised thrust command to a raw LEDC duty.
//
// TODO(bench): PER-MOTOR THRUST CALIBRATION CURVE.
// This is a straight linear map, which is what you asked for as a starting point. Real
// coreless motors are distinctly non-linear near the bottom of their range (they do not
// produce usable thrust below roughly 10-15% duty) and no two motors match.
// To do this properly: measure thrust vs duty for each motor on a scale, then replace this
// with a per-motor lookup table or polynomial. Until then, expect the drone to need trim.
//
// There is deliberately no battery-voltage compensation here: this airframe has no ADC divider
// on the pack, so there is nothing to measure. As the pack sags, the same command produces less
// thrust and the drone will need more stick towards the end of a flight - the cascade's
// integrators absorb the slow part of that on their own.
static uint32_t motor_thrust_to_duty(float thrust) {
    thrust = clampf(thrust, 0.0f, 1.0f);

    return (uint32_t)lroundf(thrust * (float)MOTOR_PWM_MAX_DUTY);
}

esp_err_t motor_set_thrust(int motor_index, float thrust) {
    if (motor_index < 0 || motor_index >= MOTOR_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!driver_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    return motor_write_duty(motor_index, motor_thrust_to_duty(thrust));
}

esp_err_t motor_set_thrust_all(const float thrust[MOTOR_COUNT]) {
    if (thrust == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!driver_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Keep going after a failure so that one dead channel does not leave the other three
    // holding their previous command, then report the first error to the caller.
    esp_err_t first_error = ESP_OK;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        esp_err_t error = motor_write_duty(i, motor_thrust_to_duty(thrust[i]));
        if (error != ESP_OK && first_error == ESP_OK) {
            first_error = error;
        }
    }

    return first_error;
}

esp_err_t motor_set_raw_duty(int motor_index, uint32_t duty) {
    if (motor_index < 0 || motor_index >= MOTOR_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!driver_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (duty > MOTOR_PWM_MAX_DUTY) {
        duty = MOTOR_PWM_MAX_DUTY;
    }

    return motor_write_duty(motor_index, duty);
}

esp_err_t motor_all_stop(void) {
    if (!driver_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Every channel is attempted even if an earlier one failed - this is the safety path and
    // a partial stop is much worse than a reported error.
    esp_err_t first_error = ESP_OK;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        esp_err_t error = motor_write_duty(i, 0);
        if (error != ESP_OK && first_error == ESP_OK) {
            first_error = error;
        }
    }

    return first_error;
}

uint32_t motor_get_last_duty(int motor_index) {
    if (motor_index < 0 || motor_index >= MOTOR_COUNT) {
        return 0;
    }

    return last_duty[motor_index];
}
