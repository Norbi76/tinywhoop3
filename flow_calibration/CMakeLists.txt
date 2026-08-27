# Top-level project file for the OPTICAL FLOW calibration bench tool.
#
# NOT drone firmware. Runs on a spare ESP32-S3 with nothing attached but the PMW3901, and
# produces one number: FLOW_COUNTS_PER_RAD for ../flight_controller/components/nav_estimator/
# nav_estimator.c, which is currently the guess 500.0f.
#
# Same reuse trick as ../tof_calibration: EXTRA_COMPONENT_DIRS points at the ONE component
# directory, not at flight_controller/components/ (which would drag in flight_control and
# pid_registry, which require telemetry_uart from ../shared_components and would fail to
# configure).

cmake_minimum_required(VERSION 3.22)

set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/../flight_controller/components/pmw3901_driver)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# ---------------------------------------------------------------------------
# SPI PIN OVERRIDES - SET THESE TO MATCH YOUR SPARE BOARD.
#
# Unlike the ToF, pmw3901_driver CREATES its own SPI bus, so the pins are compile-time #defines
# inside the driver rather than something main.c can choose. Those defines are #ifndef-guarded,
# so the values below override them for THIS build only - the drone's firmware is untouched and
# still compiles with its own defaults.
#
# Set for a XIAO ESP32-S3, per documents/XIAO_ESP32-S3_front_pinout.png. That board breaks out
# only 11 GPIOs - 1..9, 43, 44 - so the driver's own defaults (SCLK 18, CS 10) are NOT wirable
# on it. These are the board's hardware SPI pads plus a free GPIO for chip select:
#
#     GPIO7  = pad D8   (SPI0_SCK on the silkscreen)
#     GPIO8  = pad D9   (SPI0_MISO)
#     GPIO9  = pad D10  (SPI0_MOSI)
#     GPIO4  = pad D3   any free pin; CS is bit-banged by the driver, not by the SPI peripheral
#
# The "SPI0" labels are Seeed's Arduino naming. On the ESP32-S3, SPI0/SPI1 belong to flash and
# PSRAM; the driver uses SPI2_HOST and the GPIO matrix routes it to these pads.
#
# Avoid the strapping pins (0, 3, 45, 46) and the native-USB pins (19, 20).
# Nothing about the measured constant depends on the pin choice - change freely for another board.
#
# main.c prints these back at boot, so the serial log always shows what was really compiled in.
# ---------------------------------------------------------------------------
idf_build_set_property(COMPILE_DEFINITIONS "PMW3901_PIN_SCLK=7" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "PMW3901_PIN_MISO=8" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "PMW3901_PIN_MOSI=9" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "PMW3901_PIN_CS=4"   APPEND)

project(flow_calibration)
