// gs_link.c - the UDP socket, the outbound batch, and the uplink queue.
//
// WHAT THIS FILE DOES
//   gs_link_start()          mutex + queue + socket + receive task. Call AFTER the Wi-Fi AP is up.
//   gs_link_rx_task()        blocks on recvfrom, latches the peer, splits the datagram.
//   gs_link_handle_datagram() walks N concatenated records and queues each for the drone.
//   gs_link_forward()        appends one UART frame to the outbound batch (does NOT send).
//   gs_link_flush()          sends the batch, if a peer is known.
//   gs_link_pop_uplink()     the UART TX task's drain.
//
// THE LOCKING MAP - three pieces of shared state, three different answers
//   g_batch      touched only by forward()/flush(), both on the UART RX task -> NO LOCK NEEDED.
//   g_peer       written here, read by flush() on the UART RX task -> MUTEX (a torn sockaddr_in
//                would send telemetry to an address that never existed).
//   g_uplink_queue  FreeRTOS queue, zero-timeout on both ends -> the queue IS the synchronisation.
//
// THE PRIORITY CHOICE (4, below both UART tasks) IS A SAFETY PROPERTY: a ground station that has
// stopped reading must never be able to delay a control frame. Losing tuning data always beats
// losing the link that flies the drone.
//
// SILENCE IS NORMAL HERE. No peer yet -> drop the batch, no log. sendto() fails -> ignored, no
// log. Both are the ordinary state when nobody is tuning, and logging either would put a console
// line on every flush.

#include "gs_link.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include <errno.h>
#include <string.h>

static const char *TAG = "GS_LINK";

#define GS_LINK_TASK_STACK_SIZE 4096

// Below both UART tasks. A ground station that has stopped reading must never delay a control
// frame, and losing tuning data is always preferable to losing the link that flies the drone.
#define GS_LINK_TASK_PRIORITY 4
#define GS_LINK_TASK_CORE 0

// Batch size, chosen to stay under a 1500-byte Ethernet MTU once the UDP and IP headers are on
// (28 bytes) with room to spare, so nothing ever fragments. 1200 bytes holds 31 debug records.
#define GS_LINK_BATCH_SIZE 1200

// Wire record header: msg_type(1) | payload_len(1) | seq(2). See gs_link.h.
#define GS_LINK_RECORD_HEADER_SIZE 4

// Largest datagram accepted from the ground station. The uplink is small - a select is 8 bytes
// on the wire, a gain set 29 - so this is generous already.
#define GS_LINK_RX_BUFFER_SIZE 512

// Uplink queue depth. A tuning session sends bursts - switching a tab produces a select, a gain
// read-back and sometimes an inject in one datagram - so this holds several bursts while the
// 50 Hz TX task works through them.
#define GS_LINK_UPLINK_QUEUE_LENGTH 16

// One queued uplink message, carried by value so the receive task's datagram buffer can be
// reused the instant recvfrom returns.
typedef struct {
    uint8_t msg_type;
    uint8_t payload_len;
    uint8_t payload[TELEMETRY_MAX_PAYLOAD_SIZE];
} gs_link_uplink_item_t;

static int g_socket = -1;
static QueueHandle_t g_uplink_queue;

// Current peer, latched from whatever last sent us something. Written by the receive task, read
// by gs_link_flush() on the UART RX task, hence the mutex - a torn sockaddr_in would send
// telemetry to an address that never existed.
static struct sockaddr_in g_peer;
static bool g_peer_valid;
static SemaphoreHandle_t g_peer_mutex;

// Outbound batch. Touched only by gs_link_forward()/gs_link_flush(), which are both called from
// the UART RX task, so it needs no lock of its own.
static uint8_t g_batch[GS_LINK_BATCH_SIZE];
static size_t g_batch_len;

// Copies the current peer under the mutex.
// @param out Destination address.
// @return true if a peer is known and out was written.
static bool gs_link_get_peer(struct sockaddr_in *out) {
    if (g_peer_mutex == NULL) {
        return false;
    }

    // Short timeout rather than portMAX_DELAY: the only holder is a memcpy, so failing to get it
    // means something is badly wrong and blocking the UART RX task would make it worse.
    if (xSemaphoreTake(g_peer_mutex, pdMS_TO_TICKS(2)) != pdTRUE) {
        return false;
    }

    const bool valid = g_peer_valid;
    if (valid) {
        *out = g_peer;
    }

    xSemaphoreGive(g_peer_mutex);
    return valid;
}

// Latches a new peer address.
// @param addr Address the last inbound datagram came from.
static void gs_link_set_peer(const struct sockaddr_in *addr) {
    if (g_peer_mutex == NULL || xSemaphoreTake(g_peer_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }

    const bool changed = !g_peer_valid ||
                         g_peer.sin_addr.s_addr != addr->sin_addr.s_addr ||
                         g_peer.sin_port != addr->sin_port;

    g_peer = *addr;
    g_peer_valid = true;

    xSemaphoreGive(g_peer_mutex);

    if (changed) {
        // Logged only on an actual change, not per packet - the ground station sends a keepalive
        // every second and this would otherwise be a once-a-second log line forever.
        char text[16];
        inet_ntoa_r(addr->sin_addr, text, sizeof(text));
        ESP_LOGI(TAG, "Ground station peer is now %s:%u", text, ntohs(addr->sin_port));
    }
}

// Splits one inbound datagram into records and queues each for the flight controller.
// @param data Datagram bytes.
// @param length Datagram length.
static void gs_link_handle_datagram(const uint8_t *data, size_t length) {
    size_t offset = 0;

    while ((offset + GS_LINK_RECORD_HEADER_SIZE) <= length) {
        const uint8_t msg_type = data[offset];
        const uint8_t payload_len = data[offset + 1];
        // The seq field at offset + 2 is the ground station's own counter. The UART layer
        // stamps its own sequence number on transmit, so it is read past and discarded here.
        offset += GS_LINK_RECORD_HEADER_SIZE;

        if ((offset + payload_len) > length) {
            // Truncated record. The rest of the datagram is unparseable, so stop rather than
            // guessing where the next record starts.
            break;
        }

        if (payload_len > 0 && payload_len <= TELEMETRY_MAX_PAYLOAD_SIZE && g_uplink_queue != NULL) {
            gs_link_uplink_item_t item = {
                .msg_type = msg_type,
                .payload_len = payload_len,
            };
            memcpy(item.payload, &data[offset], payload_len);

            // Queued, never sent inline - see gs_link_pop_uplink(). Zero timeout so a stalled
            // TX task cannot back-pressure into this one; a dropped uplink frame costs the
            // operator one retry, which is the right trade.
            xQueueSend(g_uplink_queue, &item, 0);
        }
        // A zero-length record is the ground station's keepalive. It exists purely so the peer
        // address above stays fresh, and there is nothing to forward.

        offset += payload_len;
    }
}

// Receive task. Blocks on recvfrom, which is why this is a task rather than a poll.
static void gs_link_rx_task(void *args) {
    ESP_LOGI(TAG, "Ground station link task started on core %d", xPortGetCoreID());

    static uint8_t buffer[GS_LINK_RX_BUFFER_SIZE];

    for (;;) {
        struct sockaddr_in source;
        socklen_t source_len = sizeof(source);

        const int received = recvfrom(g_socket, buffer, sizeof(buffer), 0,
                                      (struct sockaddr *)&source, &source_len);
        if (received < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // ANY inbound packet latches the sender, before the contents are even looked at. This is
        // what makes the link self-healing: restart the ground station, or move it to a new IP
        // after a Wi-Fi drop, and telemetry follows it without the drone being touched.
        gs_link_set_peer(&source);

        gs_link_handle_datagram(buffer, (size_t)received);
    }
}

esp_err_t gs_link_start(void) {
    g_peer_mutex = xSemaphoreCreateMutex();
    if (g_peer_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create the peer mutex");
        return ESP_ERR_NO_MEM;
    }

    g_uplink_queue = xQueueCreate(GS_LINK_UPLINK_QUEUE_LENGTH, sizeof(gs_link_uplink_item_t));
    if (g_uplink_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create the uplink queue");
        return ESP_ERR_NO_MEM;
    }

    g_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (g_socket < 0) {
        ESP_LOGE(TAG, "Failed to create the UDP socket: errno %d", errno);
        return ESP_FAIL;
    }

    struct sockaddr_in bind_address = {
        .sin_family = AF_INET,
        .sin_port = htons(GS_LINK_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(g_socket, (struct sockaddr *)&bind_address, sizeof(bind_address)) < 0) {
        ESP_LOGE(TAG, "Failed to bind UDP port %d: errno %d", GS_LINK_PORT, errno);
        close(g_socket);
        g_socket = -1;
        return ESP_FAIL;
    }

    const BaseType_t result = xTaskCreatePinnedToCore(
        gs_link_rx_task, "GS_LINK_RX", GS_LINK_TASK_STACK_SIZE, NULL,
        GS_LINK_TASK_PRIORITY, NULL, GS_LINK_TASK_CORE);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create the ground station task");
        close(g_socket);
        g_socket = -1;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Ground station link listening on UDP %d", GS_LINK_PORT);

    return ESP_OK;
}

void gs_link_forward(const telemetry_message_t *msg) {
    if (msg == NULL || g_socket < 0) {
        return;
    }

    const uint8_t payload_len = msg->header.payload_len;
    const size_t needed = GS_LINK_RECORD_HEADER_SIZE + payload_len;

    if (needed > sizeof(g_batch)) {
        return;   // cannot ever fit, and would otherwise loop flushing an empty batch
    }

    if ((g_batch_len + needed) > sizeof(g_batch)) {
        gs_link_flush();
    }

    g_batch[g_batch_len++] = msg->header.msg_type;
    g_batch[g_batch_len++] = payload_len;

    // The ESP32-S3 is little-endian, so a straight copy of the 16-bit sequence number already
    // matches the little-endian field the ground station unpacks.
    memcpy(&g_batch[g_batch_len], &msg->header.seq, sizeof(msg->header.seq));
    g_batch_len += sizeof(msg->header.seq);

    if (payload_len > 0) {
        memcpy(&g_batch[g_batch_len], msg->payload, payload_len);
        g_batch_len += payload_len;
    }
}

void gs_link_flush(void) {
    if (g_batch_len == 0) {
        return;
    }

    struct sockaddr_in peer;
    if (!gs_link_get_peer(&peer)) {
        // No ground station has ever spoken to us, so there is nowhere to send this. Dropped
        // silently and on purpose: this is the normal state whenever nobody is tuning.
        g_batch_len = 0;
        return;
    }

    // The return value is deliberately ignored. This is telemetry over UDP on a link that may
    // well be down; a failed send is not an error worth reporting at this rate, and logging it
    // would put a log line on the console for every flush.
    sendto(g_socket, g_batch, g_batch_len, 0, (struct sockaddr *)&peer, sizeof(peer));

    g_batch_len = 0;
}

bool gs_link_pop_uplink(uint8_t *msg_type, void *payload, uint8_t *payload_len) {
    if (msg_type == NULL || payload == NULL || payload_len == NULL || g_uplink_queue == NULL) {
        return false;
    }

    gs_link_uplink_item_t item;
    if (xQueueReceive(g_uplink_queue, &item, 0) != pdTRUE) {
        return false;
    }

    *msg_type = item.msg_type;
    *payload_len = item.payload_len;
    memcpy(payload, item.payload, item.payload_len);

    return true;
}
