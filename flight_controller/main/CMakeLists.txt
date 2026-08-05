idf_component_register(SRCS "main.c" "sensor_task.c"
                    INCLUDE_DIRS "."
                    REQUIRES imu_driver telemetry_uart motor_driver attitude_estimator
                             nav_estimator flight_control esp_timer)
