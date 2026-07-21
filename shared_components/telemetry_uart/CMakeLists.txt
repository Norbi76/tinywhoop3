idf_component_register(SRCS "telemetry_uart.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_uart
                    REQUIRES esp_driver_gpio)
