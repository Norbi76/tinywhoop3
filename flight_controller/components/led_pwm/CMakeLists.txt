idf_component_register(SRCS "src/led_pwm.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_driver_ledc)
