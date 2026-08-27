# wifi_ap - the SoftAP the pilot connects to. Must be up before http_server and gs_link.
idf_component_register(SRCS "wifi_ap.c"
                    INCLUDE_DIRS "include"
                    # wifi_ap.h exposes only esp_err_t, so the whole Wi-Fi stack stays private.
                    PRIV_REQUIRES esp_wifi esp_netif esp_event nvs_flash)
