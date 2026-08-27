# pmw3901_driver - PixArt optical flow sensor on SPI2. Consumed by nav_estimator.
# esp_driver_gpio is needed for the MANUAL chip select line (hardware CS cannot hold CS low
# across the PMW3901's two-part register read). NOTE: this driver's init sequence is still a
# stub - see the file header in pmw3901_driver.c.
idf_component_register(SRCS "pmw3901_driver.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_spi esp_driver_gpio)
