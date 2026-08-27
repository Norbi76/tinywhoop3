# control_state - held buttons -> flight setpoints, plus the cached flight controller status.
# Written by http_server (intent) and uart_link (integration + status); depends on neither, so the
# dependency arrows all point INTO this component and there are no cycles.
idf_component_register(SRCS "control_state.c"
                    INCLUDE_DIRS "include"
                    # PUBLIC: control_state.h exposes telemetry_control_payload_t and
                    # telemetry_status_payload_t in its API.
                    REQUIRES telemetry_uart
                    PRIV_REQUIRES esp_timer)
