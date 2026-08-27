# attitude_estimator - complementary filter, IMU samples -> roll/pitch/yaw + body rates.
# REQUIRES (not PRIV_REQUIRES) imu_driver because attitude_update() takes an imu_physical_data_t
# in its public signature, so every caller needs imu_driver.h too.
idf_component_register(SRCS "attitude_estimator.c"
                    INCLUDE_DIRS "include"
                    REQUIRES imu_driver)
