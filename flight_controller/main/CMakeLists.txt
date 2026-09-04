# main - app_main(), the three FreeRTOS tasks, and the 100 Hz sensor task.
# Depends on essentially everything, because this is where the components are wired together.
# INCLUDE_DIRS "." rather than an include/ subdirectory: sensor_task.h is only consumed inside
# this component, so there is nothing to export.
idf_component_register(SRCS "main.c" "sensor_task.c"
                    INCLUDE_DIRS "."
                    REQUIRES imu_driver telemetry_uart motor_driver attitude_estimator
                             nav_estimator flight_control pid_registry esp_timer battery_monitor)
