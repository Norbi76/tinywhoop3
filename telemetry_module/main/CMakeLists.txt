idf_component_register(SRCS "main.c"
                            "wifi_ap.c"
                            "control_state.c"
                            "uart_link.c"
                            "camera_sd.c"
                            "http_server.c"
                    INCLUDE_DIRS "."
                    REQUIRES telemetry_uart esp_wifi esp_netif esp_event nvs_flash
                             esp_http_server esp_driver_uart esp_timer
                             fatfs esp_driver_sdmmc sdmmc
                    # The dashboard is compiled into the binary rather than served off the SD
                    # card, so it works with no card fitted and cannot go stale relative to
                    # the firmware.
                    EMBED_FILES "www/index.html")
