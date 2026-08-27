# motor_driver - four brushed motors on LEDC PWM. Output stage only, no mixing logic.
# Depends on nothing in this repo: flight_control depends on it, not the other way round.
idf_component_register(SRCS "motor_driver.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_ledc)
