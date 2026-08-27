# vl53l1x_driver - ToF rangefinder. Three layers, only two of which are ours:
#   vl53l1x_driver.c / vl53l1_platform.c  = project code
#   st_uld/                               = ST's vendored Ultra Lite Driver, do not edit
# This file's job is to make the vendored half OPTIONAL, so the firmware still builds and flies
# (in angle mode, altitude loop disengaged) when ST's sources are absent.
#
# The ST Ultra Lite Driver sources are not redistributed in this repo. Drop them into
# st_uld/ (see st_uld/README.md) and they get compiled in automatically; without them the
# component still builds and vl53l1x_init() reports ESP_ERR_NOT_SUPPORTED.

set(VL53L1X_SRCS "vl53l1x_driver.c" "vl53l1_platform.c")
set(VL53L1X_INCLUDES "include")

set(ULD_DIR "${CMAKE_CURRENT_LIST_DIR}/st_uld")

if(EXISTS "${ULD_DIR}/VL53L1X_api.c" AND EXISTS "${ULD_DIR}/VL53L1X_calibration.c")
    message(STATUS "vl53l1x_driver: ST ULD sources found, building with hardware support")
    list(APPEND VL53L1X_SRCS "st_uld/VL53L1X_api.c" "st_uld/VL53L1X_calibration.c")
    # st_uld comes first so that if ST's archive ships its own vl53l1_types.h it wins over
    # the minimal shim in include/.
    list(INSERT VL53L1X_INCLUDES 0 "st_uld")
else()
    message(WARNING "vl53l1x_driver: ST ULD sources NOT found in ${ULD_DIR} - ToF sensor disabled. See st_uld/README.md")
endif()

idf_component_register(SRCS ${VL53L1X_SRCS}
                    INCLUDE_DIRS ${VL53L1X_INCLUDES}
                    REQUIRES esp_driver_i2c esp_driver_gpio)

if(EXISTS "${ULD_DIR}/VL53L1X_api.c" AND EXISTS "${ULD_DIR}/VL53L1X_calibration.c")
    target_compile_definitions(${COMPONENT_LIB} PUBLIC VL53L1X_ULD_PRESENT)
    # ST's sources are third-party and are not warning-clean against IDF's -Werror settings.
    set_source_files_properties(
        st_uld/VL53L1X_api.c st_uld/VL53L1X_calibration.c
        PROPERTIES COMPILE_FLAGS "-Wno-error -Wno-unused-but-set-variable -Wno-unused-variable")
endif()
