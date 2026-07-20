idf_component_register(SRCS "web_page.c"
                    INCLUDE_DIRS "include"
                    REQUIRES telemetry_uart esp_wifi esp_netif nvs_flash)
