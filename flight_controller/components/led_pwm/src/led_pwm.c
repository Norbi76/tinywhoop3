#include "led_pwm.h"
#include "esp_log.h"
#include "driver/ledc.h"

static const char *TAG = "led_pwm";

/*
 * Multi-channel LED PWM (LEDC) helper.
 * Default configuration uses 4 LEDC channels and default GPIOs below.
 * Edit GPIO assignments to match your hardware.
 */
#define LEDC_NUM_CHANNELS 4

/* Default GPIO pins for 4 LEDs - change as needed for your board */
#define LED_GPIO0 1 //j1.x
#define LED_GPIO1 2 //j1.y
#define LED_GPIO2 3 //j2.x
#define LED_GPIO3 4 //j2.y

#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY          (5000)

/* Map our logical channels 0..3 to LEDC channel enums */
static const ledc_channel_t ledc_channels[LEDC_NUM_CHANNELS] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3,
};

static const int led_gpios[LEDC_NUM_CHANNELS] = { LED_GPIO0, LED_GPIO1, LED_GPIO2, LED_GPIO3 };

static ledc_timer_config_t s_timer_cfg;

void led_pwm_init(void)
{
    /* configure one timer for all channels */
#if defined(LEDC_HIGH_SPEED_MODE)
    ledc_mode_t mode = LEDC_HIGH_SPEED_MODE;
#else
    ledc_mode_t mode = LEDC_LOW_SPEED_MODE;
#endif
    s_timer_cfg.speed_mode = mode;
    s_timer_cfg.timer_num = LEDC_TIMER_0;
    s_timer_cfg.duty_resolution = LEDC_DUTY_RES;
    s_timer_cfg.freq_hz = LEDC_FREQUENCY;
    s_timer_cfg.clk_cfg = LEDC_AUTO_CLK;

    esp_err_t err = ledc_timer_config(&s_timer_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %d", err);
    }

    for (int i = 0; i < LEDC_NUM_CHANNELS; ++i) {
        ledc_channel_config_t ch = {
            .gpio_num = led_gpios[i],
            .speed_mode = mode,
            .channel = ledc_channels[i],
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = s_timer_cfg.timer_num,
            .duty = 0,
            .hpoint = 0,
        };
        err = ledc_channel_config(&ch);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ledc_channel_config failed for channel %d (gpio %d): %d", i, led_gpios[i], err);
        }
    }
}

static uint32_t norm_to_duty(float normY)
{
    if (normY < -1.0f) normY = -1.0f;
    if (normY >  1.0f) normY =  1.0f;
    float v = (normY + 1.0f) * 0.5f;
    uint32_t max_duty = (1 << LEDC_DUTY_RES) - 1;
    return (uint32_t)(v * (float)max_duty + 0.5f);
}

void led_pwm_set_channel_brightness(int channel, float normY)
{
    if (channel < 0 || channel >= LEDC_NUM_CHANNELS) return;
#if defined(LEDC_HIGH_SPEED_MODE)
    ledc_mode_t mode = LEDC_HIGH_SPEED_MODE;
#else
    ledc_mode_t mode = LEDC_LOW_SPEED_MODE;
#endif
    uint32_t duty = norm_to_duty(normY);
    ledc_set_duty(mode, ledc_channels[channel], duty);
    ledc_update_duty(mode, ledc_channels[channel]);
}

void led_pwm_set_all(float ch0, float ch1, float ch2, float ch3)
{
    led_pwm_set_channel_brightness(0, ch0);
    led_pwm_set_channel_brightness(1, ch1);
    led_pwm_set_channel_brightness(2, ch2);
    led_pwm_set_channel_brightness(3, ch3);
}
