# main - app_main() only. Requires every component because this is where they are wired together.
# Note the dashboard's index.html is embedded by http_server's own CMakeLists, not here.
idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES control_state uart_link telemetry_uart camera_sd wifi_ap http_server gs_link )
