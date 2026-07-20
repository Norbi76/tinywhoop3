#include "web_page.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#define WEB_SERVER_PORT 80
#define WEB_TASK_STACK_SIZE 8192
#define WEB_TASK_PRIORITY 4

static const char *TAG = "WebPage";

typedef struct {
	telemetry_imu_payload_t imu;
	telemetry_battery_payload_t battery;
	telemetry_control_payload_t last_control_rx;
	telemetry_msg_type_t last_msg_type;
	uint32_t received_count;
	bool has_imu;
	bool has_battery;
	bool has_control;
} telemetry_dashboard_state_t;

static telemetry_dashboard_state_t g_dashboard_state;
static SemaphoreHandle_t g_dashboard_mutex;
static bool g_wifi_started;

static const char *WEB_PAGE_HTML =
	"<!DOCTYPE html><html><head><meta charset='utf-8'>"
	"<meta name='viewport' content='width=device-width,initial-scale=1'>"
	"<title>Telemetry Dashboard</title>"
	"<style>body{font-family:Arial,sans-serif;background:#0f172a;color:#e2e8f0;margin:0;padding:24px}"
	".card{max-width:900px;margin:0 auto;background:#111827;border:1px solid #334155;border-radius:16px;padding:24px;box-shadow:0 12px 30px rgba(0,0,0,.25)}"
	"h1{margin-top:0} .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:16px}"
	".tile{background:#1e293b;border-radius:14px;padding:16px} .label{color:#94a3b8;font-size:12px;text-transform:uppercase;letter-spacing:.08em}"
	".value{font-size:22px;margin-top:8px;font-variant-numeric:tabular-nums} .muted{color:#94a3b8}"
	"</style></head><body><div class='card'><h1>Telemetry Dashboard</h1>"
	"<p class='muted'>Connected to ESP32-S3 access point. Live data updates every 500 ms.</p>"
	"<div class='grid'>"
	"<div class='tile'><div class='label'>Last message</div><div class='value' id='lastType'>-</div></div>"
	"<div class='tile'><div class='label'>Received count</div><div class='value' id='count'>0</div></div>"
	"<div class='tile'><div class='label'>IMU</div><div class='value' id='imu'>Waiting...</div></div>"
	"<div class='tile'><div class='label'>Battery</div><div class='value' id='battery'>Waiting...</div></div>"
	"<div class='tile'><div class='label'>Last control RX</div><div class='value' id='control'>Waiting...</div></div>"
	"</div></div><script>"
	"async function update(){const r=await fetch('/data');const d=await r.json();"
	"document.getElementById('lastType').textContent=d.last_type;"
	"document.getElementById('count').textContent=d.received_count;"
	"document.getElementById('imu').textContent=d.has_imu ? `acc ${d.imu.acc_x.toFixed(2)}, ${d.imu.acc_y.toFixed(2)}, ${d.imu.acc_z.toFixed(2)} | gyro ${d.imu.gyro_x.toFixed(2)}, ${d.imu.gyro_y.toFixed(2)}, ${d.imu.gyro_z.toFixed(2)} | temp ${d.imu.temperature.toFixed(2)} C` : 'Waiting...';"
	"document.getElementById('battery').textContent=d.has_battery ? `voltage ${d.battery.battery_voltage.toFixed(2)} V | current ${d.battery.battery_current.toFixed(2)} A | percent ${d.battery.battery_percent}%` : 'Waiting...';"
	"document.getElementById('control').textContent=d.has_control ? `roll ${d.control.roll_setpoint.toFixed(2)} pitch ${d.control.pitch_setpoint.toFixed(2)} yaw ${d.control.yaw_setpoint.toFixed(2)} thr ${d.control.throttle.toFixed(2)} armed ${d.control.armed} mode ${d.control.flight_mode}` : 'Waiting...';}"
	"update();setInterval(update,500);</script></body></html>";

static const char *msg_type_to_string(telemetry_msg_type_t msg_type) {
	switch (msg_type) {
		case TELEMETRY_MSG_IMU: return "IMU";
		case TELEMETRY_MSG_BATTERY: return "BATTERY";
		case TELEMETRY_MSG_STATUS: return "STATUS";
		case TELEMETRY_MSG_CONTROL: return "CONTROL";
		case TELEMETRY_MSG_ARMING: return "ARMING";
		case TELEMETRY_MSG_MODE: return "MODE";
		default: return "UNKNOWN";
	}
}

static void dashboard_state_init(void) {
	if (g_dashboard_mutex == NULL) {
		g_dashboard_mutex = xSemaphoreCreateMutex();
	}
}

static void dashboard_state_to_json(char *buffer, size_t buffer_size) {
	telemetry_dashboard_state_t snapshot = {0};

	if (g_dashboard_mutex != NULL && xSemaphoreTake(g_dashboard_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
		snapshot = g_dashboard_state;
		xSemaphoreGive(g_dashboard_mutex);
	}

	snprintf(buffer, buffer_size,
			 "{"
			 "\"last_type\":\"%s\"," 
			 "\"received_count\":%lu,"
			 "\"has_imu\":%s,"
			 "\"has_battery\":%s,"
			 "\"has_control\":%s,"
			 "\"imu\":{\"acc_x\":%.3f,\"acc_y\":%.3f,\"acc_z\":%.3f,\"gyro_x\":%.3f,\"gyro_y\":%.3f,\"gyro_z\":%.3f,\"temperature\":%.3f},"
			 "\"battery\":{\"battery_voltage\":%.3f,\"battery_current\":%.3f,\"battery_percent\":%u},"
			 "\"control\":{\"roll_setpoint\":%.3f,\"pitch_setpoint\":%.3f,\"yaw_setpoint\":%.3f,\"throttle\":%.3f,\"armed\":%u,\"flight_mode\":%u}"
			 "}",
			 msg_type_to_string(snapshot.last_msg_type),
			 (unsigned long)snapshot.received_count,
			 snapshot.has_imu ? "true" : "false",
			 snapshot.has_battery ? "true" : "false",
			 snapshot.has_control ? "true" : "false",
			 snapshot.imu.acc_x, snapshot.imu.acc_y, snapshot.imu.acc_z,
			 snapshot.imu.gyro_x, snapshot.imu.gyro_y, snapshot.imu.gyro_z,
			 snapshot.imu.temperature,
			 snapshot.battery.battery_voltage, snapshot.battery.battery_current, snapshot.battery.battery_percent,
			 snapshot.last_control_rx.roll_setpoint, snapshot.last_control_rx.pitch_setpoint, snapshot.last_control_rx.yaw_setpoint,
			 snapshot.last_control_rx.throttle, snapshot.last_control_rx.armed, snapshot.last_control_rx.flight_mode);
}

static void send_http_response(int client_fd, const char *content_type, const char *body) {
	char header[256];
	int body_len = (int)strlen(body);
	int header_len = snprintf(header, sizeof(header),
							  "HTTP/1.1 200 OK\r\n"
							  "Content-Type: %s\r\n"
							  "Content-Length: %d\r\n"
							  "Connection: close\r\n\r\n",
							  content_type, body_len);
	send(client_fd, header, header_len, 0);
	send(client_fd, body, body_len, 0);
}

static void send_404_response(int client_fd) {
	const char *body = "Not Found";
	char header[256];
	int header_len = snprintf(header, sizeof(header),
							  "HTTP/1.1 404 Not Found\r\n"
							  "Content-Type: text/plain\r\n"
							  "Content-Length: %d\r\n"
							  "Connection: close\r\n\r\n",
							  (int)strlen(body));
	send(client_fd, header, header_len, 0);
	send(client_fd, body, strlen(body), 0);
}

static void wifi_ap_init(void) {
	if (g_wifi_started) {
		return;
	}

	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ESP_ERROR_CHECK(nvs_flash_init());
	} else {
		ESP_ERROR_CHECK(ret);
	}

	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_ap();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	wifi_config_t wifi_config = {
		.ap = {
			.ssid = "TelemetryAP",
			.ssid_len = 0,
			.channel = 1,
			.password = "telemetry123",
			.max_connection = 4,
			.authmode = WIFI_AUTH_WPA2_PSK,
		},
	};

	if (strlen((const char *)wifi_config.ap.password) == 0) {
		wifi_config.ap.authmode = WIFI_AUTH_OPEN;
	}

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());

	esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
	if (ap_netif != NULL) {
		esp_netif_ip_info_t ip_info;
		if (esp_netif_get_ip_info(ap_netif, &ip_info) == ESP_OK) {
			ESP_LOGI(TAG, "AP started. SSID:%s Password:%s IP:" IPSTR,
					 (const char *)wifi_config.ap.ssid,
					 (const char *)wifi_config.ap.password,
					 IP2STR(&ip_info.ip));
		}
	}

	g_wifi_started = true;
}

static void web_task(void *pvParameters) {
	ESP_LOGI(TAG, "Web task started on core %d", xPortGetCoreID());

	wifi_ap_init();

	int server_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
	if (server_fd < 0) {
		ESP_LOGE(TAG, "Unable to create socket: %d", errno);
		vTaskDelete(NULL);
		return;
	}

	int enable = 1;
	setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));

	struct sockaddr_in server_addr = {0};
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(WEB_SERVER_PORT);

	if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
		ESP_LOGE(TAG, "Socket bind failed: %d", errno);
		close(server_fd);
		vTaskDelete(NULL);
		return;
	}

	if (listen(server_fd, 4) != 0) {
		ESP_LOGE(TAG, "Socket listen failed: %d", errno);
		close(server_fd);
		vTaskDelete(NULL);
		return;
	}

	ESP_LOGI(TAG, "Web server listening on port %d", WEB_SERVER_PORT);

	while (true) {
		struct sockaddr_in client_addr;
		socklen_t client_len = sizeof(client_addr);
		int client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_len);
		if (client_fd < 0) {
			ESP_LOGW(TAG, "Accept failed: %d", errno);
			vTaskDelay(pdMS_TO_TICKS(100));
			continue;
		}

		char request[512] = {0};
		int received = recv(client_fd, request, sizeof(request) - 1, 0);
		if (received <= 0) {
			close(client_fd);
			continue;
		}

		if (strstr(request, "GET /data ") != NULL) {
			char json[768];
			dashboard_state_to_json(json, sizeof(json));
			send_http_response(client_fd, "application/json", json);
		} else if (strstr(request, "GET / ") != NULL || strstr(request, "GET /HTTP") != NULL) {
			send_http_response(client_fd, "text/html", WEB_PAGE_HTML);
		} else {
			send_404_response(client_fd);
		}

		close(client_fd);
	}
}

esp_err_t web_page_init(void) {
	dashboard_state_init();
	return ESP_OK;
}

esp_err_t web_page_start_task(UBaseType_t core_id) {
	BaseType_t result = xTaskCreatePinnedToCore(
		web_task,
		"WEB_TASK",
		WEB_TASK_STACK_SIZE,
		NULL,
		WEB_TASK_PRIORITY,
		NULL,
		core_id);

	return (result == pdPASS) ? ESP_OK : ESP_FAIL;
}

void web_page_store_message(const telemetry_message_t *message) {
	if (message == NULL) {
		return;
	}

	if (g_dashboard_mutex == NULL || xSemaphoreTake(g_dashboard_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
		return;
	}

	g_dashboard_state.last_msg_type = (telemetry_msg_type_t)message->header.msg_type;
	g_dashboard_state.received_count++;

	switch ((telemetry_msg_type_t)message->header.msg_type) {
		case TELEMETRY_MSG_IMU:
			memcpy(&g_dashboard_state.imu, message->payload, sizeof(telemetry_imu_payload_t));
			g_dashboard_state.has_imu = true;
			break;
		case TELEMETRY_MSG_BATTERY:
			memcpy(&g_dashboard_state.battery, message->payload, sizeof(telemetry_battery_payload_t));
			g_dashboard_state.has_battery = true;
			break;
		case TELEMETRY_MSG_CONTROL:
			memcpy(&g_dashboard_state.last_control_rx, message->payload, sizeof(telemetry_control_payload_t));
			g_dashboard_state.has_control = true;
			break;
		default:
			break;
	}

	xSemaphoreGive(g_dashboard_mutex);
}
