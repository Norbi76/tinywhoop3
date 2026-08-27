# gs_link - UDP bridge tunnelling the flight controller's PID tuning traffic to the Python
# ground station. Purely a pipe: it does not interpret any of the frames it carries.
idf_component_register(SRCS "gs_link.c"
                    INCLUDE_DIRS "include"
                    # PUBLIC: gs_link.h takes a telemetry_message_t in its API.
                    REQUIRES telemetry_uart
                    # PRIVATE: the UDP socket is entirely an implementation detail. Note this
                    # component deliberately does NOT require uart_link - the dependency runs the
                    # other way, so the uplink queue lives here and uart_link drains it.
                    PRIV_REQUIRES lwip)
