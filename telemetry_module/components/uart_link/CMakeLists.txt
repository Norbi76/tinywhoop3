# uart_link - the serial link to the flight controller. Two tasks on core 0: TX at a fixed 50 Hz,
# RX continuous. This is the component that actually flies the drone.
idf_component_register(SRCS "uart_link.c"
                    INCLUDE_DIRS "include"
                    # PUBLIC: uart_link.h exposes telemetry_gains_payload_t in its API, so
                    # anyone including it needs telemetry_uart's headers too.
                    REQUIRES telemetry_uart
                    # PRIVATE: only the implementation talks to control_state and gs_link.
                    # gs_link must not require this component back - the ground station's uplink
                    # queue lives in gs_link and is drained here, precisely to keep that one-way.
                    PRIV_REQUIRES control_state gs_link)
