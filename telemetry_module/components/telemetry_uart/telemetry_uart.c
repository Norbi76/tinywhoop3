#include "telemetry_uart.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>
#include <stddef.h>
#include "esp_err.h"

#define UART_BUF_SIZE (1024 * 2)

static uart_port_t active_uart_num; 
static uint16_t sequence_counter;

uint16_t telemetry_calculate_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

esp_err_t uart_telemetry_init(const telemetry_uart_config_t *config) {
    esp_err_t error;
    
    active_uart_num = config->uart_port;
    const uart_config_t uart_cfg = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    error = uart_param_config(active_uart_num, &uart_cfg);
    if (error != ESP_OK) {
        return error;
    }

    error = uart_set_pin(active_uart_num, config->tx_pin, config->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (error != ESP_OK) {
        return error;
    }

    error = uart_driver_install(active_uart_num, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0);
    if (error != ESP_OK) {
        return error;
    }

    return ESP_OK;
}

bool uart_telemetry_read_message(telemetry_message_t *out_message) {
    telemetry_frame_header_t header;

    int rx_bytes = uart_read_bytes(active_uart_num, (uint8_t *)&header, sizeof(header), pdMS_TO_TICKS(2));
    if (rx_bytes != sizeof(header)) {
        return false;
    }

    if (header.start_byte != TELEMETRY_START_BYTE || header.version != TELEMETRY_PROTOCOL_VERSION) {
        uart_flush_input(active_uart_num);
        return false;
    }

    if (header.payload_len > TELEMETRY_MAX_PAYLOAD_SIZE) {
        uart_flush_input(active_uart_num);
        return false;
    }

    telemetry_frame_tail_t tail;
    uint8_t payload[TELEMETRY_MAX_PAYLOAD_SIZE];

    if (header.payload_len > 0) {
        rx_bytes = uart_read_bytes(active_uart_num, payload, header.payload_len, pdMS_TO_TICKS(2));
        if (rx_bytes != header.payload_len) {
            return false;
        }
    }

    rx_bytes = uart_read_bytes(active_uart_num, (uint8_t *)&tail, sizeof(tail), pdMS_TO_TICKS(2));
    if (rx_bytes != sizeof(tail)) {
        return false;
    }

    if (tail.end_byte != TELEMETRY_END_BYTE) {
        uart_flush_input(active_uart_num);
        return false;
    }

    uint8_t crc_buffer[sizeof(header) + TELEMETRY_MAX_PAYLOAD_SIZE];
    memcpy(crc_buffer, &header, sizeof(header));
    if (header.payload_len > 0) {
        memcpy(crc_buffer + sizeof(header), payload, header.payload_len);
    }

    uint16_t calculated_crc = telemetry_calculate_crc16(crc_buffer, sizeof(header) + header.payload_len);
    if (calculated_crc != tail.crc16) {
        return false;
    }

    out_message->header = header;
    if (header.payload_len > 0) {
        memcpy(out_message->payload, payload, header.payload_len);
    }
    out_message->tail = tail;
    return true;
}

esp_err_t uart_telemetry_send_message(telemetry_msg_type_t msg_type, const void *payload, uint8_t payload_len) {
    if (payload_len > TELEMETRY_MAX_PAYLOAD_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    telemetry_frame_header_t header = {
        .start_byte = TELEMETRY_START_BYTE,
        .version = TELEMETRY_PROTOCOL_VERSION,
        .msg_type = (uint8_t)msg_type,
        .payload_len = payload_len,
        .seq = sequence_counter++,
    };

    telemetry_frame_tail_t tail = {
        .crc16 = 0,
        .end_byte = TELEMETRY_END_BYTE,
    };

    uint8_t crc_buffer[sizeof(header) + TELEMETRY_MAX_PAYLOAD_SIZE];
    memcpy(crc_buffer, &header, sizeof(header));
    if (payload_len > 0 && payload != NULL) {
        memcpy(crc_buffer + sizeof(header), payload, payload_len);
    }

    tail.crc16 = telemetry_calculate_crc16(crc_buffer, sizeof(header) + payload_len);

    int written = uart_write_bytes(active_uart_num, (const char *)&header, sizeof(header));
    if (written < 0) {
        return ESP_FAIL;
    }

    if (payload_len > 0 && payload != NULL) {
        written = uart_write_bytes(active_uart_num, (const char *)payload, payload_len);
        if (written < 0) {
            return ESP_FAIL;
        }
    }

    written = uart_write_bytes(active_uart_num, (const char *)&tail, sizeof(tail));
    return (written < 0) ? ESP_FAIL : ESP_OK;
}