# camera_sd - camera capture to SD card, plus the QVGA viewfinder. Runs on core 1, alone.
# Completely independent of flight: nothing here is on the control path and every failure here is
# non-fatal. See README.md for the exposure-locking rationale.
idf_component_register(SRCS "camera_sd.c"
                    INCLUDE_DIRS "include"
                    # camera_sd.h only needs stdint/stdbool/esp_err/freertos, so nothing below
                    # belongs in the public interface. esp32-camera itself arrives through
                    # idf_component.yml rather than being listed here.
                    PRIV_REQUIRES esp_driver_sdmmc fatfs sdmmc)
