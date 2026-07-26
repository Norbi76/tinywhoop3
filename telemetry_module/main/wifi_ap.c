#include "wifi_ap.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_mac.h"     // MACSTR / MAC2STR
#include "nvs_flash.h"
#include <string.h>

static const char *TAG = "WIFI_AP";

static bool ap_started;
static int client_count;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    if (event_base != WIFI_EVENT) {
        return;
    }

    switch (event_id) {
        case WIFI_EVENT_AP_STACONNECTED: {
            wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
            client_count++;
            ESP_LOGI(TAG, "Station connected: " MACSTR " (aid=%d, %d total)",
                     MAC2STR(event->mac), event->aid, client_count);
            break;
        }
        case WIFI_EVENT_AP_STADISCONNECTED: {
            wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
            if (client_count > 0) {
                client_count--;
            }
            // Worth a warning: losing the station mid-flight is how the control link dies.
            ESP_LOGW(TAG, "Station disconnected: " MACSTR " (aid=%d, %d left)",
                     MAC2STR(event->mac), event->aid, client_count);
            break;
        }
        default:
            break;
    }
}

esp_err_t wifi_ap_init(void) {
    if (ap_started) {
        return ESP_OK;
    }

    esp_err_t error;

    // --- NVS (the Wi-Fi driver stores calibration data here) ------------------
    error = nvs_flash_init();
    if (error == ESP_ERR_NVS_NO_FREE_PAGES || error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        error = nvs_flash_init();
    }
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(error));
        return error;
    }

    error = esp_netif_init();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(error));
        return error;
    }

    error = esp_event_loop_create_default();
    if (error != ESP_OK && error != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(error));
        return error;
    }

    esp_netif_create_default_wifi_ap();

    const wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    error = esp_wifi_init(&init_config);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(error));
        return error;
    }

    error = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                &wifi_event_handler, NULL, NULL);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register Wi-Fi event handler: %s", esp_err_to_name(error));
        return error;
    }

    wifi_config_t ap_config = {
        .ap = {
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = WIFI_AP_CHANNEL,
            .max_connection = WIFI_AP_MAX_CONNECTIONS,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };
    strncpy((char *)ap_config.ap.ssid, WIFI_AP_SSID, sizeof(ap_config.ap.ssid) - 1);
    strncpy((char *)ap_config.ap.password, WIFI_AP_PASSWORD, sizeof(ap_config.ap.password) - 1);

    // WPA2 needs at least 8 characters; fall back to an open AP rather than failing to start,
    // because an unreachable drone is worse than an unsecured one.
    if (strlen(WIFI_AP_PASSWORD) < 8) {
        ESP_LOGW(TAG, "Password shorter than 8 characters - falling back to an OPEN network");
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    error = esp_wifi_set_mode(WIFI_MODE_AP);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode failed: %s", esp_err_to_name(error));
        return error;
    }

    error = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config failed: %s", esp_err_to_name(error));
        return error;
    }

    error = esp_wifi_start();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(error));
        return error;
    }

    // ---------------------------------------------------------------------
    // POWER SAVE OFF - this is the whole reason this function exists rather than using the
    // stock example. With the default WIFI_PS_MIN_MODEM the radio sleeps between beacons and
    // buffers frames until the next one, which adds tens of milliseconds of jitter to every
    // control POST. On a drone being flown by hand that is the difference between responsive
    // and unflyable. Must be called AFTER esp_wifi_start().
    // ---------------------------------------------------------------------
    error = esp_wifi_set_ps(WIFI_PS_NONE);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable Wi-Fi power save: %s", esp_err_to_name(error));
        return error;
    }

    // ---------------------------------------------------------------------
    // TRANSMIT POWER: TURNED DOWN.
    //
    // The default is the maximum the radio will do, which is sized for tens of metres through
    // walls. This drone is flown across a room, and transmit power is the single largest
    // current draw the radio has - it matters on a battery this small, and it matters most
    // exactly when the viewfinder is on and the radio is transmitting continuously.
    //
    // Note this is NOT the same trade as power save. WIFI_PS_NONE above keeps the radio awake
    // so control latency stays low; this only reduces how loudly it shouts. Latency is
    // unaffected.
    //
    // TODO(bench): raise CAM_WIFI_TX_POWER_QDBM if you see the dashboard stuttering or the
    // link dropping at the range you actually fly. Units are 0.25 dBm, so 52 = 13 dBm, and the
    // usable range is roughly 8 (2 dBm) to 80 (20 dBm). Try 60 (15 dBm) before going higher.
    // ---------------------------------------------------------------------
    error = esp_wifi_set_max_tx_power(WIFI_TX_POWER_QDBM);
    if (error != ESP_OK) {
        // Not fatal: a louder radio than intended still flies, it just costs more battery.
        ESP_LOGW(TAG, "Could not set Wi-Fi TX power: %s (continuing at default)",
                 esp_err_to_name(error));
    } else {
        int8_t actual = 0;
        esp_wifi_get_max_tx_power(&actual);
        ESP_LOGI(TAG, "Wi-Fi TX power set to %d (%.1f dBm)", actual, actual / 4.0f);
    }

    ap_started = true;

    esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (ap_netif != NULL) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(ap_netif, &ip_info) == ESP_OK) {
            ESP_LOGI(TAG, "SoftAP up: SSID '%s', channel %d, power save OFF",
                     WIFI_AP_SSID, WIFI_AP_CHANNEL);
            ESP_LOGI(TAG, "Dashboard at http://" IPSTR "/", IP2STR(&ip_info.ip));
        }
    }

    return ESP_OK;
}

bool wifi_ap_is_started(void) {
    return ap_started;
}

int wifi_ap_get_client_count(void) {
    return client_count;
}
