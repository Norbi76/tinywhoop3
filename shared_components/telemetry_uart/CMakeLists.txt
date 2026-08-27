# telemetry_uart - the UART wire protocol shared by BOTH firmware projects.
#
# This component is not vendored into either project. Each project's top-level CMakeLists.txt adds
# ../shared_components to EXTRA_COMPONENT_DIRS, so both builds compile this one copy on disk.
# esp_timer is required for the resync reader's total-time deadline; the rest is the UART driver.
idf_component_register(SRCS "telemetry_uart.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_uart esp_driver_gpio esp_timer)
