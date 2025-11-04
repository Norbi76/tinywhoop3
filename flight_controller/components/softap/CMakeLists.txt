idf_component_register(SRCS "softap.c" "web_pages.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_http_server
                       PRIV_REQUIRES esp_event esp_netif esp_wifi nvs_flash)
