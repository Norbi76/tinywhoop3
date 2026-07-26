idf_component_register(SRCS "uart_link.c"
                    INCLUDE_DIRS "include"
                    # PUBLIC: uart_link.h exposes telemetry_gains_payload_t in its API, so
                    # anyone including it needs telemetry_uart's headers too.
                    REQUIRES telemetry_uart
                    # PRIVATE: only the implementation talks to control_state.
                    PRIV_REQUIRES control_state)
