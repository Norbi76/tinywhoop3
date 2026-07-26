idf_component_register(SRCS "http_server.c"
                    INCLUDE_DIRS "include"
                    # http_server.h exposes only esp_err_t. Everything it drives is an
                    # implementation detail, so none of it is re-exported to callers.
                    PRIV_REQUIRES esp_http_server control_state camera_sd uart_link wifi_ap
                    # The dashboard is compiled into the binary rather than served off the SD
                    # card, so it works with no card fitted and cannot go stale relative to
                    # the firmware.
                    EMBED_FILES "www/index.html")
