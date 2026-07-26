idf_component_register(SRCS "control_state.c"
                    INCLUDE_DIRS "include"
                    # PUBLIC: control_state.h exposes telemetry_control_payload_t and
                    # telemetry_status_payload_t in its API.
                    REQUIRES telemetry_uart
                    PRIV_REQUIRES esp_timer)
