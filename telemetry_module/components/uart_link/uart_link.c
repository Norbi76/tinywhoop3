// uart_link.c - the TX and RX tasks that carry everything to and from the flight controller.
//
// WHAT THIS FILE DOES
//   uart_link_init()        creates the gains queue and brings up the UART driver.
//   uart_link_tx_task()     fixed 50 Hz: integrate -> control frame -> gains -> GS uplink.
//   uart_link_rx_task()     5 ms wakeups: bounded drain -> cache status -> forward -> flush.
//   uart_link_queue_gains() the HTTP handler's non-blocking way to get a gain frame sent.
//
// THE THREE "WHY IS IT BOUNDED" CONSTANTS, all the same reasoning in different places
//   GAINS_PER_CYCLE (2) and UPLINK_PER_CYCLE (4) cap how much optional traffic can follow the
//   control frame in one cycle, so tuning bursts cannot delay the frame that keeps the drone
//   armed. RX_DRAIN_PER_CYCLE (64) caps the other direction, so a flood of PID debug frames
//   cannot monopolise the RX task - while still being ~20x more headroom than the worst case.
//
// WHY THE RX TASK BATCHES ITS FORWARDING
//   gs_link_forward() only appends to a buffer; gs_link_flush() is what sends. One datagram per
//   drain instead of one per frame, because at 500 Hz the latter is 500 packets/second and the
//   SoftAP chokes on packet RATE long before bitrate.
//
// LOGGING IS RATE-LIMITED to once a second on both send-failure paths. At 50 Hz an unconditional
// ESP_LOGE would saturate the console UART on its own.

#include "uart_link.h"
#include "control_state.h"
#include "gs_link.h"
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

// How many ground-station uplink frames to relay per TX cycle. Same reasoning as GAINS_PER_CYCLE:
// a burst of tuning traffic must not push the control frame late.
#define UPLINK_PER_CYCLE 4

// Frames drained per RX wakeup before the batch is flushed to the ground station. At a 5 ms
// wakeup this is 12800 frames/second of headroom against a worst case near 550.
#define RX_DRAIN_PER_CYCLE 64

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

        // --- Piggyback any queued ground-station uplink --------------------
        // These come from gs_link's UDP task, which must not call uart_telemetry_send_message()
        // itself - see gs_link_pop_uplink(). Unlogged: a tuning session can push several of
        // these per second and the console UART is a shared resource.
        uint8_t uplink_type;
        uint8_t uplink_len;
        uint8_t uplink_payload[TELEMETRY_MAX_PAYLOAD_SIZE];
        for (int i = 0; i < UPLINK_PER_CYCLE; i++) {
            if (!gs_link_pop_uplink(&uplink_type, uplink_payload, &uplink_len)) {
                break;
            }

            error = uart_telemetry_send_message((telemetry_msg_type_t)uplink_type,
                                                uplink_payload, uplink_len);
            if (error != ESP_OK) {
                static int64_t last_uplink_log_tick;
                const int64_t now_tick = (int64_t)xTaskGetTickCount();
                if ((now_tick - last_uplink_log_tick) > pdMS_TO_TICKS(1000)) {
                    ESP_LOGW(TAG, "Uplink frame send failed: %s", esp_err_to_name(error));
                    last_uplink_log_tick = now_tick;
                }
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
        // Bounded drain rather than one frame per wakeup. With PID debug frames arriving at up
        // to 500 Hz on top of the 50 Hz status stream, a one-frame-per-cycle reader would fall
        // permanently behind and the dashboard's attitude readout would age without bound.
        int drained = 0;
        while (drained < RX_DRAIN_PER_CYCLE) {
            // The resynchronising reader scans for a start byte, validates the length, the end
            // byte and the CRC16, and on any failure discards just the bytes it consumed rather
            // than flushing the buffer. A flush here would take good control-side frames with it.
            if (!uart_telemetry_read_message_resync(&message, 2)) {
                break;   // nothing more queued right now
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

            // Every frame is forwarded to the ground station, including the ones the switch
            // above ignores - the dashboard and the ground station are independent consumers and
            // neither filters for the other. This only buffers; the datagram goes out at flush.
            gs_link_forward(&message);
            drained++;
        }

        // One datagram per drain rather than one per frame. At 500 Hz the latter would be 500
        // packets/second and the SoftAP would choke on the packet rate long before the bitrate.
        gs_link_flush();

        vTaskDelay(pdMS_TO_TICKS(5));
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
