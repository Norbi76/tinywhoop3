# nav_estimator - ToF range + optical flow -> altitude, climb rate, body velocity.
# Both sensor drivers are public REQUIRES because their result structs (vl53l1x_result_t,
# pmw3901_motion_t) appear in this component's function signatures. esp_timer provides the
# wall-clock timestamps behind the 200 ms validity timeouts.
idf_component_register(SRCS "nav_estimator.c"
                    INCLUDE_DIRS "include"
                    REQUIRES vl53l1x_driver pmw3901_driver esp_timer)
