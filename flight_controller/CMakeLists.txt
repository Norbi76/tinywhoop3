# Top-level project file for the FLIGHT CONTROLLER firmware (runs on the drone itself).
# Built independently of ../telemetry_module - there is no top-level build for the repo.
#
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

# This is what makes shared_components/telemetry_uart visible as a component WITHOUT vendoring it:
# both firmware projects point at the same directory on disk, so the wire protocol has exactly one
# copy and cannot drift between the two boards. See shared_components/telemetry_uart/README.md.
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/../shared_components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(flight_controller)
