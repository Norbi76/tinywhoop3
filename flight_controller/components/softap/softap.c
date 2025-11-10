/* softap component implementation */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "softap.h"
#include "web_pages.h"
#include "sdkconfig.h"
#include "esp_heap_caps.h"
#include "driver/ledc.h"
#include "cJSON.h"

static const char *TAG = "softap_component";

/* PWM / LEDC configuration — change these to match your hardware */
#define LEDC_OUTPUT_IO          (2) // GPIO pin connected to LED
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_TIMER              LEDC_TIMER_0
#if defined(LEDC_HIGH_SPEED_MODE)
#define LEDC_MODE               LEDC_HIGH_SPEED_MODE
#else
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#endif
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY          (5000)

/* Initialize LED PWM (LEDC) */
static void ledc_init_pwm(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %d", err);
    }

    ledc_channel_config_t ledc_channel = {
        .gpio_num       = LEDC_OUTPUT_IO,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed: %d", err);
    }
}

/* Map normalized joystick Y [-1..1] to PWM duty [0..max] and apply it */
static void set_led_brightness(float normY)
{
    if (normY < -1.0f) normY = -1.0f;
    if (normY >  1.0f) normY =  1.0f;
    /* map -1..1 -> 0..1 */
    float v = (normY + 1.0f) * 0.5f;
    uint32_t max_duty = (1 << LEDC_DUTY_RES) - 1;
    uint32_t duty = (uint32_t)(v * (float)max_duty + 0.5f);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

/* Parse joystick JSON (supports either {"x":..,"y":..} or
   {"j1": {"x":..,"y":..}, "j2":{...}}) and act on it.
   Maps j1.y (or single y) to LED brightness. */
static void process_joystick_json(const char *buf)
{
    if (!buf) return;
    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        ESP_LOGW(TAG, "JSON parse failed");
        return;
    }

    /* Try old single-object format first */
    cJSON *xitem = cJSON_GetObjectItem(root, "x");
    cJSON *yitem = cJSON_GetObjectItem(root, "y");
    if (cJSON_IsNumber(xitem) && cJSON_IsNumber(yitem)) {
        float jx = (float)xitem->valuedouble;
        float jy = (float)yitem->valuedouble;
        ESP_LOGI(TAG, "Parsed single joystick JSON: x=%f y=%f", jx, jy);
        set_led_brightness(jy);
        cJSON_Delete(root);
        return;
    }

    /* Try combined format with j1/j2 */
    cJSON *j1 = cJSON_GetObjectItem(root, "j1");
    cJSON *j2 = cJSON_GetObjectItem(root, "j2");
    if (j1 && cJSON_IsObject(j1)) {
        cJSON *jx = cJSON_GetObjectItem(j1, "x");
        cJSON *jy = cJSON_GetObjectItem(j1, "y");
        if (cJSON_IsNumber(jx) && cJSON_IsNumber(jy)) {
            float j1x = (float)jx->valuedouble;
            float j1y = (float)jy->valuedouble;
            ESP_LOGI(TAG, "Parsed j1: x=%f y=%f", j1x, j1y);
            /* use j1.y for brightness mapping */
            set_led_brightness(j1y);
        }
    }
    if (j2 && cJSON_IsObject(j2)) {
        cJSON *jx2 = cJSON_GetObjectItem(j2, "x");
        cJSON *jy2 = cJSON_GetObjectItem(j2, "y");
        if (cJSON_IsNumber(jx2) && cJSON_IsNumber(jy2)) {
            float j2x = (float)jx2->valuedouble;
            float j2y = (float)jy2->valuedouble;
            ESP_LOGI(TAG, "Parsed j2: x=%f y=%f", j2x, j2y);
            /* if you want to act on j2, do it here */
        }
    }

    cJSON_Delete(root);
}

/* HTTP GET handler */
static esp_err_t hello_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
}

static const httpd_uri_t hello = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = hello_get_handler,
    .user_ctx  = NULL
};

/* WebSocket handler: compile only when WS support is enabled in menuconfig.
    Support two possible config macro names used across ESP-IDF versions/repos: 
    - CONFIG_ESP_HTTP_SERVER_WS (preferred)
    - CONFIG_HTTPD_WS_SUPPORT (older or alternate)
*/
#if defined(CONFIG_ESP_HTTP_SERVER_WS) || defined(CONFIG_HTTPD_WS_SUPPORT)
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method != HTTP_GET) return ESP_FAIL;
    /* Validate that this request includes a websocket upgrade handshake.
       Some clients or accidental HTTP GETs may hit /ws without the required
       Upgrade/Connection headers which causes the httpd websocket helper to
       warn about "No handshake performed". Reject such non-handshake GETs
       with a 400 to avoid noisy warnings. */
    char upgrade_val[32] = {0};
    char conn_val[32] = {0};
    if (httpd_req_get_hdr_value_str(req, "Upgrade", upgrade_val, sizeof(upgrade_val)) != ESP_OK ||
        httpd_req_get_hdr_value_str(req, "Connection", conn_val, sizeof(conn_val)) != ESP_OK) {
        ESP_LOGW(TAG, "WebSocket handshake headers missing (Upgrade/Connection) - rejecting request");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Not a websocket handshake");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "WebSocket connection opened");

    while (true) {
        httpd_ws_frame_t ws_pkt;
        memset(&ws_pkt, 0, sizeof(ws_pkt));
        esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
        if (ret != ESP_OK) {
            ESP_LOGI(TAG, "httpd_ws_recv_frame failed: %d", ret);
            break;
        }
        if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
            ESP_LOGI(TAG, "WS close frame received");
            break;
        }
        if (ws_pkt.len > 0) {
            if (ws_pkt.type == HTTPD_WS_TYPE_TEXT) {
                char *buf = malloc(ws_pkt.len + 1);
                if (!buf) { ESP_LOGE(TAG, "malloc failed for ws payload"); break; }
                ws_pkt.payload = (uint8_t*)buf;
                ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
                if (ret != ESP_OK) { ESP_LOGI(TAG, "ws recv payload error: %d", ret); free(buf); break; }
                buf[ws_pkt.len] = '\0';
                ESP_LOGI(TAG, "WS received: %s", buf);
                float jx=0, jy=0;
                /* parse JSON (single or combined) and handle joystick(s) */
                process_joystick_json(buf);
                free(buf);
            } else {
                ESP_LOGI(TAG, "WS received non-text frame (type=%d len=%d)", ws_pkt.type, ws_pkt.len);
                uint8_t *tmp = malloc(ws_pkt.len);
                if (tmp) {
                    ws_pkt.payload = tmp;
                    ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
                    free(tmp);
                    if (ret != ESP_OK) { ESP_LOGI(TAG, "ws recv binary payload error: %d", ret); break; }
                } else { ESP_LOGE(TAG, "malloc failed for binary ws payload"); break; }
            }
        }
    }

    ESP_LOGI(TAG, "WebSocket connection closed");
    return ESP_OK;
}
#endif

/* WebSocket URI; is registered only when WS support is enabled */
#if defined(CONFIG_ESP_HTTP_SERVER_WS) || defined(CONFIG_HTTPD_WS_SUPPORT)
static const httpd_uri_t ws_uri = {
    .uri = "/ws",
    .method = HTTP_GET,
    .handler = ws_handler,
    .user_ctx = NULL
};
#endif

/* POST fallback */
static esp_err_t joy_post_handler(httpd_req_t *req)
{
    int total_len = req->content_len;
    if (total_len <= 0) { httpd_resp_send_404(req); return ESP_FAIL; }

    char *buf = malloc(total_len + 1);
    if (!buf) { ESP_LOGE(TAG, "malloc failed in joy_post_handler"); httpd_resp_send_500(req); return ESP_FAIL; }

    int received = 0;
    while (received < total_len) {
        int ret = httpd_req_recv(req, buf + received, total_len - received);
        if (ret <= 0) { free(buf); ESP_LOGE(TAG, "httpd_req_recv failed: %d", ret); httpd_resp_send_500(req); return ESP_FAIL; }
        received += ret;
    }
    buf[total_len] = '\0';
    ESP_LOGI(TAG, "POST /joy payload: %s", buf);
    /* parse JSON body (single or combined) */
    process_joystick_json(buf);
    free(buf);

    const char resp[] = "OK";
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, resp, sizeof(resp)-1);
    return ESP_OK;
}

static const httpd_uri_t joy_uri = {
    .uri = "/joy",
    .method = HTTP_POST,
    .handler = joy_post_handler,
    .user_ctx = NULL
};

static httpd_handle_t start_webserver_internal(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    ESP_LOGI(TAG, "Starting HTTP Server");
    if (httpd_start(&server, &config) == ESP_OK) {
    httpd_register_uri_handler(server, &hello);
#if defined(CONFIG_ESP_HTTP_SERVER_WS) || defined(CONFIG_HTTPD_WS_SUPPORT)
    ESP_LOGI(TAG, "WebSocket support enabled in build; registering /ws URI");
    httpd_register_uri_handler(server, &ws_uri);
#else
    ESP_LOGI(TAG, "WebSocket support NOT enabled in build");
#endif
    httpd_register_uri_handler(server, &joy_uri);
    }
    return server;
}

void softap_init(void)
{
    const char *ssid = "TinyWhoopAP";
    const char *password = "tinywhoop123";
    /* initialize LED PWM hardware so joystick Y can control brightness */
    ledc_init_pwm();

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "",
            .ssid_len = 0,
            .channel = 1,
            .password = "",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };

    strncpy((char*)wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid));
    strncpy((char*)wifi_config.ap.password, password, sizeof(wifi_config.ap.password));
    wifi_config.ap.ssid_len = strlen(ssid);

    if (strlen(password) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi SoftAP started. SSID:%s password:%s", ssid, password);
    ESP_LOGI(TAG, "Connect a client and open http://192.168.4.1/");
}

httpd_handle_t softap_start(void)
{
    return start_webserver_internal();
}

void softap_stop(httpd_handle_t server)
{
    if (server) {
        httpd_stop(server);
    }
}
