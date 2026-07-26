idf_component_register(SRCS "camera_sd.c"
                    INCLUDE_DIRS "include"
                    # camera_sd.h only needs stdint/stdbool/esp_err/freertos, so nothing below
                    # belongs in the public interface. esp32-camera itself arrives through
                    # idf_component.yml rather than being listed here.
                    PRIV_REQUIRES esp_driver_sdmmc fatfs sdmmc)
