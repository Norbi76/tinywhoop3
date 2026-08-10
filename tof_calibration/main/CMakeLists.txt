idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES vl53l1x_driver esp_driver_i2c)
