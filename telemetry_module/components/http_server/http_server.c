#include "http_server.h"
#include "control_state.h"
#include "camera_sd.h"
#include "uart_link.h"
#include "wifi_ap.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

// Upper bound on a QVGA preview JPEG. Static rather than on the stack: the HTTP task stack
// is nowhere near big enough for this, and only one request is ever served at a time.
#define CAM_PREVIEW_MAX_JPEG (32 * 1024)

static const char *TAG = "HTTP";

// index.html is compiled into the binary by EMBED_FILES in main/CMakeLists.txt, so there is
// no filesystem dependency for serving the dashboard - it works even with no SD card.
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

// The dashboard's POST bodies are all small JSON objects.
#define MAX_BODY_LEN 512

static httpd_handle_t server_handle;

// Reads a request body into `buffer` and NUL-terminates it.
// @return ESP_OK on success, or an error if the body was too large or the socket died.
static esp_err_t read_body(httpd_req_t *req, char *buffer, size_t buffer_size) {
    const size_t total = req->content_len;

    if (total >= buffer_size) {
        ESP_LOGW(TAG, "Request body too large: %u bytes", (unsigned)total);
        return ESP_ERR_INVALID_SIZE;
    }

    size_t received = 0;
    while (received < total) {
        int chunk = httpd_req_recv(req, buffer + received, total - received);
        if (chunk <= 0) {
            if (chunk == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            return ESP_FAIL;
        }
        received += chunk;
    }

    buffer[received] = '\0';
    return ESP_OK;
}

// Minimal JSON scraping.
//
// Deliberately not a real JSON parser: the only producer is our own dashboard, the payloads
// are three or four flat keys, and pulling in cJSON to parse {"roll_left":true} would cost
// heap and parse time on the hot 20 Hz path for no benefit. If the dashboard ever grows
// nested objects, replace these with cJSON rather than extending them.

// Returns true if "key":true appears in the body.
static bool json_flag(const char *body, const char *key) {
    char pattern[48];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);

    const char *found = strstr(body, pattern);
    if (found == NULL) {
        return false;
    }

    // Look at the value that follows the colon. "true" is the only thing we treat as set.
    const char *colon = strchr(found, ':');
    if (colon == NULL) {
        return false;
    }

    return strncmp(colon + 1, "true", 4) == 0 ||
           strncmp(colon + 1, " true", 5) == 0;
}

// Reads a numeric value for `key`. Returns `fallback` if absent or unparseable.
static double json_number(const char *body, const char *key, double fallback) {
    char pattern[48];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);

    const char *found = strstr(body, pattern);
    if (found == NULL) {
        return fallback;
    }

    const char *colon = strchr(found, ':');
    if (colon == NULL) {
        return fallback;
    }

    char *end = NULL;
    const double value = strtod(colon + 1, &end);
    if (end == colon + 1) {
        return fallback;   // nothing numeric there
    }

    return value;
}

static esp_err_t send_json_ok(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

// GET / - the dashboard.
static esp_err_t handler_root(httpd_req_t *req) {
    const size_t length = index_html_end - index_html_start;

    httpd_resp_set_type(req, "text/html");
    // No caching: during development the page changes on every flash, and a stale cached
    // dashboard controlling a drone is a genuinely bad failure mode.
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    return httpd_resp_send(req, (const char *)index_html_start, length);
}

// POST /api/input - held button state, 20 Hz.
// This is the hot path. It parses eight booleans, stores them, and returns.
static esp_err_t handler_input(httpd_req_t *req) {
    char body[MAX_BODY_LEN];
    if (read_body(req, body, sizeof(body)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
        return ESP_FAIL;
    }

    control_buttons_t buttons = {
        .pitch_forward = json_flag(body, "pitch_forward"),
        .pitch_back    = json_flag(body, "pitch_back"),
        .roll_left     = json_flag(body, "roll_left"),
        .roll_right    = json_flag(body, "roll_right"),
        .yaw_left      = json_flag(body, "yaw_left"),
        .yaw_right     = json_flag(body, "yaw_right"),
        .throttle_up   = json_flag(body, "throttle_up"),
        .throttle_down = json_flag(body, "throttle_down"),
    };

    // Only stores state. The ramp/decay integration happens on the 50 Hz UART TX task, so the
    // setpoints advance at a fixed rate regardless of how irregularly these POSTs arrive.
    control_state_set_buttons(&buttons);

    return send_json_ok(req);
}

// POST /api/arm - arm, disarm or kill.
static esp_err_t handler_arm(httpd_req_t *req) {
    char body[MAX_BODY_LEN];
    if (read_body(req, body, sizeof(body)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
        return ESP_FAIL;
    }

    if (json_flag(body, "kill")) {
        control_state_set_kill(true);
        ESP_LOGW(TAG, "KILL from dashboard");
        return send_json_ok(req);
    }

    const bool armed = json_flag(body, "armed");

    // Any explicit arm/disarm also clears a previous kill request, which is what lets the
    // pilot recover without power cycling. The flight controller's own latch still requires
    // kill to be released AND arm to be low before it will leave the killed state.
    control_state_set_kill(false);
    control_state_set_arm(armed);

    return send_json_ok(req);
}

// POST /api/mode - flight mode and hold toggle.
static esp_err_t handler_mode(httpd_req_t *req) {
    char body[MAX_BODY_LEN];
    if (read_body(req, body, sizeof(body)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
        return ESP_FAIL;
    }

    control_state_set_hold(json_flag(body, "hold"));

    const int mode = (int)json_number(body, "mode", -1.0);
    if (mode >= 0 && mode <= TELEMETRY_MODE_POS_HOLD) {
        control_state_set_mode((uint8_t)mode);
    }

    return send_json_ok(req);
}

// POST /api/capture - queue a photo.
//
// This handler does exactly one thing: push a token onto a queue. It does NOT grab the frame
// and it does NOT write the file. Both of those happen on the capture task on core 1. An SD
// write can block for hundreds of milliseconds, and doing that here would stall the HTTP
// server and therefore the 20 Hz control POSTs.
static esp_err_t handler_capture(httpd_req_t *req) {
    const esp_err_t error = camera_sd_request_capture();

    // Tell the control state so the capture bit rides along on the next control frame - the
    // flight controller can then log which photos line up with which attitude.
    if (error == ESP_OK) {
        control_state_flag_capture();
    }

    httpd_resp_set_type(req, "application/json");

    if (error == ESP_ERR_INVALID_STATE) {
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"camera or sd unavailable\"}");
    }
    if (error == ESP_ERR_NO_MEM) {
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"queue full\"}");
    }

    return httpd_resp_sendstr(req, "{\"ok\":true,\"queued\":true}");
}

// POST /api/gains - live PID tuning for one loop.
static esp_err_t handler_gains(httpd_req_t *req) {
    char body[MAX_BODY_LEN];
    if (read_body(req, body, sizeof(body)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
        return ESP_FAIL;
    }

    const int loop_id = (int)json_number(body, "loop", -1.0);
    if (loop_id < 0 || loop_id >= TELEMETRY_LOOP_COUNT) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad loop id");
        return ESP_FAIL;
    }

    const telemetry_gains_payload_t gains = {
        .loop_id = (uint8_t)loop_id,
        .kp = (float)json_number(body, "kp", 0.0),
        .ki = (float)json_number(body, "ki", 0.0),
        .kd = (float)json_number(body, "kd", 0.0),
    };

    // Queued, not sent inline: it goes out on the next 50 Hz TX cycle so it cannot delay a
    // control frame.
    const esp_err_t error = uart_link_queue_gains(&gains);

    httpd_resp_set_type(req, "application/json");
    if (error != ESP_OK) {
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"queue full\"}");
    }

    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

// GET /api/status - everything the dashboard displays.
static esp_err_t handler_status(httpd_req_t *req) {
    telemetry_status_payload_t status;
    const bool have_status = control_state_get_status(&status);
    const int32_t status_age_ms = control_state_status_age_ms();

    camera_sd_stats_t camera_stats;
    camera_sd_get_stats(&camera_stats);

    // The link counts as up only if a status frame arrived recently. A stale frame from ten
    // seconds ago must not read as a healthy link.
    const bool link = have_status && (status_age_ms >= 0) && (status_age_ms < 500);

    char json[640];
    snprintf(json, sizeof(json),
             "{"
             "\"link\":%s,"
             "\"armed\":%s,"
             "\"killed\":%s,"
             "\"attitude_init\":%s,"
             "\"alt_valid\":%s,"
             "\"vel_valid\":%s,"
             "\"mode\":%u,"
             "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
             "\"altitude\":%.3f,\"climb\":%.3f,"
             "\"vx\":%.3f,\"vy\":%.3f,"
             "\"battery\":%.2f,"
             "\"throttle_trim\":%.3f,"
             "\"loop_hz\":%u,"
             "\"motors\":[%.3f,%.3f,%.3f,%.3f],"
             "\"status_age_ms\":%ld,"
             "\"clients\":%d,"
             "\"shots_requested\":%lu,\"shots_written\":%lu,\"shots_failed\":%lu,"
             "\"camera_ok\":%s,\"sd_ok\":%s,"
             "\"rx_frames\":%lu,\"tx_frames\":%lu"
             "}",
             link ? "true" : "false",
             (link && status.armed) ? "true" : "false",
             (status.flags & TELEMETRY_STATUS_FLAG_KILLED) ? "true" : "false",
             (status.flags & TELEMETRY_STATUS_FLAG_ATTITUDE_INIT) ? "true" : "false",
             (status.flags & TELEMETRY_STATUS_FLAG_ALT_VALID) ? "true" : "false",
             (status.flags & TELEMETRY_STATUS_FLAG_VEL_VALID) ? "true" : "false",
             status.flight_mode,
             status.roll, status.pitch, status.yaw,
             status.altitude, status.climb_rate,
             status.velocity_x, status.velocity_y,
             status.battery_voltage,
             (double)control_state_get_throttle_trim(),
             status.loop_hz,
             status.motor[0], status.motor[1], status.motor[2], status.motor[3],
             (long)status_age_ms,
             wifi_ap_get_client_count(),
             (unsigned long)camera_stats.requested,
             (unsigned long)camera_stats.written,
             (unsigned long)camera_stats.failed,
             camera_sd_camera_ok() ? "true" : "false",
             camera_sd_sd_ok() ? "true" : "false",
             (unsigned long)uart_link_get_rx_count(),
             (unsigned long)uart_link_get_tx_count());

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    return httpd_resp_sendstr(req, json);
}


// Toggles the viewfinder. Returns immediately; the capture task applies the change.
static esp_err_t handler_preview(httpd_req_t *req) {
    char body[128];
    const int length = read_body(req, body, sizeof(body));

    httpd_resp_set_type(req, "application/json");
    if (length < 0) {
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"bad body\"}");
    }

    const bool enable = json_flag(body, "enable");
    const esp_err_t error = camera_sd_set_preview(enable);
    if (error != ESP_OK) {
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"camera unavailable\"}");
    }

    return httpd_resp_sendstr(req, enable ? "{\"ok\":true,\"preview\":true}"
                                          : "{\"ok\":true,\"preview\":false}");
}

// Serves the most recent viewfinder frame as a plain JPEG.
//
// Deliberately ONE FRAME PER REQUEST rather than an MJPEG stream. ESP-IDF's HTTP server
// services requests serially on a single task, so a long-lived multipart response would sit in
// that task and delay the dashboard's 20 Hz control POSTs for as long as the stream was open.
// Discrete requests keep the control path responsive, and they degrade gracefully: if the link
// slows down the viewfinder just gets choppier while control carries on unaffected.
static esp_err_t handler_preview_jpg(httpd_req_t *req) {
    static uint8_t frame[CAM_PREVIEW_MAX_JPEG];

    const size_t length = camera_sd_copy_preview(frame, sizeof(frame));
    if (length == 0) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"no preview frame\"}");
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)frame, length);
}

static const httpd_uri_t uri_handlers[] = {
    { .uri = "/",            .method = HTTP_GET,  .handler = handler_root },
    { .uri = "/api/input",   .method = HTTP_POST, .handler = handler_input },
    { .uri = "/api/arm",     .method = HTTP_POST, .handler = handler_arm },
    { .uri = "/api/mode",    .method = HTTP_POST, .handler = handler_mode },
    { .uri = "/api/capture", .method = HTTP_POST, .handler = handler_capture },
    { .uri = "/api/gains",   .method = HTTP_POST, .handler = handler_gains },
    { .uri = "/api/status",  .method = HTTP_GET,  .handler = handler_status },
    { .uri = "/api/preview", .method = HTTP_POST, .handler = handler_preview },
    { .uri = "/preview.jpg", .method = HTTP_GET,  .handler = handler_preview_jpg },
};

esp_err_t http_server_start(void) {
    if (server_handle != NULL) {
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 12;
    config.stack_size = 8192;

    // Core 0, alongside the UART tasks and away from the camera work on core 1.
    config.core_id = 0;

    // Below the UART tasks (7 and 6): serving the dashboard must never delay a control frame.
    config.task_priority = 5;

    // The dashboard opens several short-lived connections; purging the least recently used
    // avoids running out of sockets during a long session.
    config.lru_purge_enable = true;

    esp_err_t error = httpd_start(&server_handle, &config);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(error));
        server_handle = NULL;
        return error;
    }

    for (size_t i = 0; i < sizeof(uri_handlers) / sizeof(uri_handlers[0]); i++) {
        error = httpd_register_uri_handler(server_handle, &uri_handlers[i]);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register %s: %s", uri_handlers[i].uri, esp_err_to_name(error));
            return error;
        }
    }

    ESP_LOGI(TAG, "HTTP server up on port %d with %u handlers",
             config.server_port, (unsigned)(sizeof(uri_handlers) / sizeof(uri_handlers[0])));

    return ESP_OK;
}

esp_err_t http_server_stop(void) {
    if (server_handle == NULL) {
        return ESP_OK;
    }

    const esp_err_t error = httpd_stop(server_handle);
    server_handle = NULL;
    return error;
}
