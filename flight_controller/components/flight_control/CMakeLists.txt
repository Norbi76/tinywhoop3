# flight_control - the cascade, the mixer, the arming state machine and the link watchdog.
# The orchestrator: it depends on every estimator and the motor driver, and main/ depends on it.
idf_component_register(SRCS "flight_control.c"
                    INCLUDE_DIRS "include"
                    REQUIRES pid_controller motor_driver attitude_estimator nav_estimator telemetry_uart esp_timer
                    # PRIVATE: the tuning registry is an implementation detail of the cascade -
                    # nothing in flight_control.h mentions it.
                    PRIV_REQUIRES pid_registry)
