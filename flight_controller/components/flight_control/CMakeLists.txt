idf_component_register(SRCS "flight_control.c"
                    INCLUDE_DIRS "include"
                    REQUIRES pid_controller motor_driver attitude_estimator nav_estimator telemetry_uart esp_timer)
