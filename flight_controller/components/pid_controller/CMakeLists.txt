# pid_controller - the generic PID primitive. Deliberately has NO dependencies: it knows nothing
# about quadcopters, sensors, FreeRTOS or this airframe, which is what lets flight_control
# instantiate it eight times with completely different units and rates.
idf_component_register(SRCS "pid_controller.c"
                    INCLUDE_DIRS "include")
