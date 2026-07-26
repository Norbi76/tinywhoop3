#include "uart_link.h"
#include "control_state.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "UART_LINK";

#define TX_TASK_STACK_SIZE 4096
#define RX_TASK_STACK_SIZE 4096

// TX sits above RX: missing a transmit slot risks the flight controller's watchdog, whereas a
// late status frame only makes the dashboard slightly stale.
#define TX_TASK_PRIORITY 7
#define RX_TASK_PRIORITY 6

#define GAINS_QUEUE_LENGTH 8

// How many gain frames to drain per TX cycle. Kept small so that a burst of slider drags
// cannot push the control frame late.
#define GAINS_PER_CYCLE 2

static QueueHandle_t gains_queue;
static uint32_t rx_frame_count;
static uint32_t tx_cycle_count;

esp_err_t uart_link_init(void) {
    gains_queue = xQueueCreate(GAINS_QUEUE_LENGTH, sizeof(telemetry_gains_payload_t));
    if (gains_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create gains queue");
        return ESP_FAIL;
    }

    const telemetry_uart_config_t uart_config = {
        .uart_port = UART_LINK_PORT,
        .tx_pin = UART_LINK_TX_PIN,
        .rx_pin = UART_LINK_RX_PIN,
        .baud_rate = UART_LINK_BAUD_RATE,
    };

    esp_err_t error = uart_telemetry_init(&uart_config);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "uart_telemetry_init failed: %s", esp_err_to_name(error));
        return error;
    }

    ESP_LOGI(TAG, "UART link up: TX=GPIO%d RX=GPIO%d @ %d baud, sending at %d Hz",
             UART_LINK_TX_PIN, UART_LINK_RX_PIN, UART_LINK_BAUD_RATE, UART_LINK_TX_HZ);

    return ESP_OK;
}

// Fixed-rate transmit task.
static void uart_link_tx_task(void *args) {
    ESP_LOGI(TAG, "UART TX task started on core %d", xPortGetCoreID());

    const TickType_t period = pdMS_TO_TICKS(1000 / UART_LINK_TX_HZ);
    TickType_t last_wake_time = xTaskGetTickCount();

    const float dt = 1.0f / (float)UART_LINK_TX_HZ;

    for (;;) {
        // Advance the ramp/decay integration and run the browser watchdog. This lives here
        // rather than in the HTTP handler so the setpoints evolve at a fixed, known rate even
        // when the browser is posting irregularly or has stopped entirely.
        control_state_update(dt);

        telemetry_control_payload_t control;
        control_state_get_frame(&control);

        esp_err_t error = uart_telemetry_send_message(TELEMETRY_MSG_CONTROL,
                                                      &control, sizeof(control));
        if (error != ESP_OK) {
            // Do not spam at 50 Hz - once a second is enough to notice.
            static int64_t last_log_tick;
            const int64_t now_tick = (int64_t)xTaskGetTickCount();
            if ((now_tick - last_log_tick) > pdMS_TO_TICKS(1000)) {
                ESP_LOGE(TAG, "Control frame send failed: %s", esp_err_to_name(error));
                last_log_tick = now_tick;
            }
        }

        // --- Piggyback any queued gain updates -----------------------------
        telemetry_gains_payload_t gains;
        for (int i = 0; i < GAINS_PER_CYCLE; i++) {
            if (xQueueReceive(gains_queue, &gains, 0) != pdTRUE) {
                break;
            }

            error = uart_telemetry_send_message(TELEMETRY_MSG_GAINS, &gains, sizeof(gains));
            if (error != ESP_OK) {
                ESP_LOGW(TAG, "Gains frame send failed: %s", esp_err_to_name(error));
            } else {
                ESP_LOGI(TAG, "Sent gains for loop %u: kp=%.5f ki=%.5f kd=%.5f",
                         gains.loop_id, (double)gains.kp, (double)gains.ki, (double)gains.kd);
            }
        }

        tx_cycle_count++;

        // vTaskDelayUntil rather than vTaskDelay: the transmit cadence must not drift with
        // however long the sends above took.
        vTaskDelayUntil(&last_wake_time, period);
    }
}

// Continuous receive task.
static void uart_link_rx_task(void *args) {
    ESP_LOGI(TAG, "UART RX task started on core %d", xPortGetCoreID());

    telemetry_message_t message;

    for (;;) {
        // The resynchronising reader scans for a start byte, validates the length, the end
        // byte and the CRC16, and on any failure discards just the bytes it consumed rather
        // than flushing the buffer. A flush here would take good control-side frames with it.
        if (!uart_telemetry_read_message_resync(&message, 20)) {
            // Nothing valid within the timeout. Yield briefly and try again.
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        switch ((telemetry_msg_type_t)message.header.msg_type) {
            case TELEMETRY_MSG_STATUS: {
                if (message.header.payload_len < sizeof(telemetry_status_payload_t)) {
                    ESP_LOGW(TAG, "Short status frame: %u bytes", message.header.payload_len);
                    break;
                }

                const telemetry_status_payload_t *status =
                    (const telemetry_status_payload_t *)message.payload;
                control_state_store_status(status);
                rx_frame_count++;
                break;
            }
            case TELEMETRY_MSG_IMU:
                // Raw IMU frames arrive at 10 Hz. The dashboard reads attitude out of the
                // status frame instead, so nothing to do here beyond not treating it as junk.
                break;
            default:
                break;
        }
    }
}

esp_err_t uart_link_start(BaseType_t core_id) {
    BaseType_t result = xTaskCreatePinnedToCore(
        uart_link_tx_task, "UART_TX", TX_TASK_STACK_SIZE, NULL, TX_TASK_PRIORITY, NULL, core_id);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UART TX task");
        return ESP_FAIL;
    }

    result = xTaskCreatePinnedToCore(
        uart_link_rx_task, "UART_RX", RX_TASK_STACK_SIZE, NULL, RX_TASK_PRIORITY, NULL, core_id);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UART RX task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t uart_link_queue_gains(const telemetry_gains_payload_t *gains) {
    if (gains == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (gains_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Zero timeout: this is called from the HTTP handler and must never block it.
    if (xQueueSend(gains_queue, gains, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Gains queue full, dropping update for loop %u", gains->loop_id);
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

uint32_t uart_link_get_rx_count(void) {
    return rx_frame_count;
}

uint32_t uart_link_get_tx_count(void) {
    return tx_cycle_count;
}
