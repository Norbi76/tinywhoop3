idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES control_state uart_link telemetry_uart camera_sd wifi_ap http_server )
