# imu_driver - MPU-9250-class 6-axis IMU on I2C0.
#
# This component OWNS the I2C master bus (imu_setup() creates it); vl53l1x_driver is a second
# device on the same bus. No dependency is declared between them - the ordering is enforced at
# runtime by main.c's imu_ready_semaphore instead.
idf_component_register(SRCS "imu_driver.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_i2c
                    )
