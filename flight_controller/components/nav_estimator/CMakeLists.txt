idf_component_register(SRCS "nav_estimator.c"
                    INCLUDE_DIRS "include"
                    REQUIRES vl53l1x_driver pmw3901_driver esp_timer)
