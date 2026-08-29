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
// WHY THE CONTROL ECHO GOES THROUGH A QUEUE INSTEAD OF STRAIGHT INTO gs_link_forward()
//   The ground station used to see only what the drone REPORTED, never what the pilot ASKED FOR,
//   because the control frame is built and sent by the TX task and forwarding happens on the RX
//   task. A flight log without the commanded setpoints cannot answer "was the pilot fighting it
//   or was it drifting on its own", so the frame is now echoed to the ground station too.
//
//   It is NOT forwarded from the TX task. gs_link.c's locking map says g_batch needs no mutex
//   *precisely because* forward() and flush() are only ever called from the RX task; calling
//   forward() from a second task would race g_batch_len and corrupt or overrun the datagram
//   buffer. So the TX task hands the payload to control_echo_queue and the RX task, which is
//   already the single writer, does the forwarding. Same shape as the gains and uplink queues.
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

// Control-frame echo to the ground station. See the header comment for why this is a queue.
//
// Depth 4 against a producer at 50 Hz (one every 20 ms) and a consumer waking every 5 ms: the RX
// task gets four chances to drain each frame, so the queue only backs up if that task has been
// starved for the better part of a tenth of a second, at which point a missing log sample is the
// least of the problems. The send is zero-timeout, so a full queue costs one echoed frame and
// never delays the control frame itself.
//
// COSTS NOTHING ON THE UART. gs_link_forward() only appends to the outbound UDP batch; the frame
// that flies the drone is already on the wire by then. On Wi-Fi it is 23 B per frame at 50 Hz =
// 1.15 kB/s, and it rides in datagrams that are being sent anyway.
#define CONTROL_ECHO_QUEUE_LENGTH 4

// How many echoes to forward per RX drain. Four covers the worst backlog the queue can hold.
#define CONTROL_ECHO_PER_CYCLE 4

static QueueHandle_t gains_queue;
static QueueHandle_t control_echo_queue;
static uint32_t rx_frame_count;
static uint32_t tx_cycle_count;
static uint32_t control_echo_dropped;

esp_err_t uart_link_init(void) {
    gains_queue = xQueueCreate(GAINS_QUEUE_LENGTH, sizeof(telemetry_gains_payload_t));
    if (gains_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create gains queue");
        return ESP_FAIL;
    }

    control_echo_queue = xQueueCreate(CONTROL_ECHO_QUEUE_LENGTH,
                                      sizeof(telemetry_control_payload_t));
    if (control_echo_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create control echo queue");
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

        // --- Echo the same frame to the ground station ----------------------
        // Queued, not forwarded here: the RX task owns gs_link's batch buffer. See the header.
        // Sent whether or not the UART send above succeeded, deliberately - a log that shows the
        // pilot commanding right while nothing reached the flight controller is exactly the
        // picture you want when working out why the drone did not respond.
        if (control_echo_queue != NULL &&
            xQueueSend(control_echo_queue, &control, 0) != pdTRUE) {
            control_echo_dropped++;
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

        // --- Fold in the TX task's control echoes ---------------------------
        // These never came off the UART; they are what the telemetry module SENT. Forwarded from
        // here rather than from the TX task so that this task stays the only writer of gs_link's
        // batch buffer - see the header comment.
        //
        // The sequence number is this echo's own counter, not the UART sequence, so the ground
        // station can spot a gap and know its flight log is missing frames rather than that the
        // pilot stopped commanding. It is deliberately allowed to wrap at 16 bits.
        if (control_echo_queue != NULL) {
            static uint16_t echo_seq;
            telemetry_message_t echo = {0};
            echo.header.msg_type = TELEMETRY_MSG_CONTROL;
            echo.header.payload_len = (uint8_t)sizeof(telemetry_control_payload_t);

            for (int i = 0; i < CONTROL_ECHO_PER_CYCLE; i++) {
                if (xQueueReceive(control_echo_queue, echo.payload, 0) != pdTRUE) {
                    break;
                }
                echo.header.seq = echo_seq++;
                gs_link_forward(&echo);
            }
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

uint32_t uart_link_get_control_echo_dropped(void) {
    return control_echo_dropped;
}
