
/* Simple ESP-IDF SoftAP + HTTP server
   Serves a "Hello World" page at http://192.168.4.1/ when connected to the AP.
*/

#include <stdio.h>
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

static const char *TAG = "softap_http";

/* HTTP GET handler */
#include "web_pages.h"


static esp_err_t hello_get_handler(httpd_req_t *req)
{
	httpd_resp_set_type(req, "text/html");
	httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
	return ESP_OK;
}

static const httpd_uri_t hello = {
	.uri       = "/",
	.method    = HTTP_GET,
	.handler   = hello_get_handler,
	.user_ctx  = NULL
};

/* WebSocket URI: receives joystick JSON from the browser */
static esp_err_t ws_handler(httpd_req_t *req)
{
	if (req->method != HTTP_GET) {
		return ESP_FAIL;
	}

	ESP_LOGI(TAG, "WebSocket connection opened");

	while (true) {
		httpd_ws_frame_t ws_pkt;
		memset(&ws_pkt, 0, sizeof(ws_pkt));

		/* First call to get frame info (type/len) */
		esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
		if (ret != ESP_OK) {
			ESP_LOGI(TAG, "httpd_ws_recv_frame failed: %d", ret);
			break;
		}

		/* handle close */
		if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
			ESP_LOGI(TAG, "WS close frame received");
			break;
		}

		if (ws_pkt.len > 0) {
			if (ws_pkt.type == HTTPD_WS_TYPE_TEXT) {
				/* receive text frame into buffer and null-terminate */
				char *buf = malloc(ws_pkt.len + 1);
				if (!buf) {
					ESP_LOGE(TAG, "malloc failed for ws payload");
					break;
				}
				ws_pkt.payload = (uint8_t*)buf;
				ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
				if (ret != ESP_OK) {
					ESP_LOGI(TAG, "ws recv payload error: %d", ret);
					free(buf);
					break;
				}
				buf[ws_pkt.len] = '\0';
				ESP_LOGI(TAG, "WS received: %s", buf);
				/* try a simple parse for JSON like {"x":0.12,"y":-0.34} */
				float jx=0, jy=0;
				if (sscanf(buf, "{\"x\":%f,\"y\":%f}", &jx, &jy) == 2) {
					ESP_LOGI(TAG, "Parsed joystick: x=%f y=%f", jx, jy);
					/* TODO: use values (e.g., drive motors) */
				}
				free(buf);
			} else {
				/* non-text frame (binary) - drain it safely */
				ESP_LOGI(TAG, "WS received non-text frame (type=%d len=%d)", ws_pkt.type, ws_pkt.len);
				uint8_t *tmp = malloc(ws_pkt.len);
				if (tmp) {
					ws_pkt.payload = tmp;
					ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
					free(tmp);
					if (ret != ESP_OK) {
						ESP_LOGI(TAG, "ws recv binary payload error: %d", ret);
						break;
					}
				} else {
					ESP_LOGE(TAG, "malloc failed for binary ws payload");
					break;
				}
			}
		}
	}

	ESP_LOGI(TAG, "WebSocket connection closed");
	return ESP_OK;
}

static const httpd_uri_t ws_uri = {
	.uri = "/ws",
	.method = HTTP_GET,
	.is_websocket = true,
	.handler = ws_handler,
	.user_ctx = NULL
};

/* HTTP POST handler to receive joystick values as JSON fallback */
static esp_err_t joy_post_handler(httpd_req_t *req)
{
	int total_len = req->content_len;
	if (total_len <= 0) {
		httpd_resp_send_404(req);
		return ESP_FAIL;
	}

	char *buf = malloc(total_len + 1);
	if (!buf) {
		ESP_LOGE(TAG, "malloc failed in joy_post_handler");
		httpd_resp_send_500(req);
		return ESP_FAIL;
	}

	int received = 0;
	while (received < total_len) {
		int ret = httpd_req_recv(req, buf + received, total_len - received);
		if (ret <= 0) {
			free(buf);
			ESP_LOGE(TAG, "httpd_req_recv failed: %d", ret);
			httpd_resp_send_500(req);
			return ESP_FAIL;
		}
		received += ret;
	}
	buf[total_len] = '\0';

	ESP_LOGI(TAG, "POST /joy payload: %s", buf);
	/* simple parse {"x":0.12,"y":-0.34} */
	float jx = 0, jy = 0;
	if (sscanf(buf, "{\"x\":%f,\"y\":%f}", &jx, &jy) == 2) {
		ESP_LOGI(TAG, "Parsed joystick POST: x=%f y=%f", jx, jy);
		/* TODO: apply joystick values */
	}

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

static httpd_handle_t start_webserver(void)
{
	httpd_handle_t server = NULL;
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();

	ESP_LOGI(TAG, "Starting HTTP Server");
	if (httpd_start(&server, &config) == ESP_OK) {
		httpd_register_uri_handler(server, &hello);
		/* register websocket URI */
		httpd_register_uri_handler(server, &ws_uri);
		/* register HTTP POST fallback for joystick */
		httpd_register_uri_handler(server, &joy_uri);
	}
	return server;
}

static void wifi_init_softap(void)
{
	const char *ssid = "TinyWhoopAP";
	const char *password = "tinywhoop123"; // must be >=8 chars for WPA2

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

void app_main(void)
{
	ESP_LOGI(TAG, "Initializing SoftAP + HTTP server example");

	wifi_init_softap();

	/* start the web server */
	start_webserver();

	/* keep running */
	while (1) {
		vTaskDelay(pdMS_TO_TICKS(10000));
	}
}
