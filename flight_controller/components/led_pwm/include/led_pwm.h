#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Initialize LED PWM hardware (LEDC)
// Initialize LED PWM hardware (LEDC) for up to 4 channels.
// By default this will use the GPIO pins defined in the implementation
// (see components/led_pwm/src/led_pwm.c). If you want different pins,
// edit that file or add a new init variant.
void led_pwm_init(void);

// Set LED brightness for a specific channel (0..3) using normalized
// input in range [-1..1]. If channel is out of range it will be ignored.
void led_pwm_set_channel_brightness(int channel, float normY);

// Convenience helper to set all four channels at once.
void led_pwm_set_all(float ch0, float ch1, float ch2, float ch3);

#ifdef __cplusplus
}
#endif
