# Top-level project file for the TOF CALIBRATION bench tool.
#
# This is NOT drone firmware. It runs on a spare ESP32-S3 with nothing attached but the VL53L1X,
# and exists to produce one number: the value that belongs in VL53L1X_OFFSET_MM in
# ../flight_controller/components/vl53l1x_driver/vl53l1x_driver.c.
#
# EXTRA_COMPONENT_DIRS points at the vl53l1x_driver component DIRECTORY ITSELF, not at the
# components/ folder that contains it. That is deliberate and load-bearing:
#   - Pointing at flight_controller/components/ would discover ALL of them, and flight_control /
#     pid_registry REQUIRE telemetry_uart from ../shared_components, which is not on this
#     project's search path. That is a configure-time failure, not a warning.
#   - IDF accepts a single component directory here, so we pull in exactly the one we want.
#
# Reusing the real component (rather than copying it) is the whole point: the offset is only
# valid for the ranging configuration it was measured under, and vl53l1x_init() applies the
# same short mode / 20 ms budget / 25 ms inter-measurement the drone actually flies with.

cmake_minimum_required(VERSION 3.22)

set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/../flight_controller/components/vl53l1x_driver)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tof_calibration)
