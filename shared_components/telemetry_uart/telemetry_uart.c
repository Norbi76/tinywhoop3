#include "telemetry_uart.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>
#include <stddef.h>
#include "esp_err.h"
#include "esp_timer.h"

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

    // Reads the header
    int rx_bytes = uart_read_bytes(active_uart_num, (uint8_t *)&header, sizeof(header), pdMS_TO_TICKS(2));
    if (rx_bytes != sizeof(header)) {
        return false;
    }

    // Validate the header, checks the start byte
    if (header.start_byte != TELEMETRY_START_BYTE) {
        uart_flush_input(active_uart_num);
        return false;
    }

    //Validate the payload length
    if (header.payload_len > TELEMETRY_MAX_PAYLOAD_SIZE) {
        uart_flush_input(active_uart_num);
        return false;
    }

    telemetry_frame_tail_t tail;
    uint8_t payload[TELEMETRY_MAX_PAYLOAD_SIZE];

    // Reads the payload if there is any
    if (header.payload_len > 0) {
        rx_bytes = uart_read_bytes(active_uart_num, payload, header.payload_len, pdMS_TO_TICKS(2));
        if (rx_bytes != header.payload_len) {
            return false;
        }
    }

    //Reads the tail
    rx_bytes = uart_read_bytes(active_uart_num, (uint8_t *)&tail, sizeof(tail), pdMS_TO_TICKS(2));
    if (rx_bytes != sizeof(tail)) {
        return false;
    }

    // Validates the tail, checks the end byte
    if (tail.end_byte != TELEMETRY_END_BYTE) {
        uart_flush_input(active_uart_num);
        return false;
    }

    // Prepare the CRC buffer to pass it on the CRC calculation function
    uint8_t crc_buffer[sizeof(header) + TELEMETRY_MAX_PAYLOAD_SIZE];
    memcpy(crc_buffer, &header, sizeof(header));
    if (header.payload_len > 0) {
        memcpy(crc_buffer + sizeof(header), payload, header.payload_len);
    }

    // Calculate the CRC for the read header and payload
    uint16_t calculated_crc = telemetry_calculate_crc16(crc_buffer, sizeof(header) + header.payload_len);
    // Check the calculated CRC against the received CRC in the tail
    if (calculated_crc != tail.crc16) {
        uart_flush_input(active_uart_num);
        return false;
    }

    //Copy the read data into the output message structure
    out_message->header = header;
    if (header.payload_len > 0) {
        memcpy(out_message->payload, payload, header.payload_len);
    }
    out_message->tail = tail;
    return true;
}

bool uart_telemetry_read_message_resync(telemetry_message_t *out_message, uint32_t timeout_ms) {
    if (out_message == NULL) {
        return false;
    }

    const int64_t deadline_us = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    while (esp_timer_get_time() < deadline_us) {
        uint8_t byte = 0;

        // Scan forward one byte at a time until a start byte turns up.
        // Everything skipped here is either line noise or the tail of a frame we already gave up on.
        if (uart_read_bytes(active_uart_num, &byte, 1, 1) != 1) {
            continue;
        }
        if (byte != TELEMETRY_START_BYTE) {
            continue;
        }

        // Start byte found. Read the rest of the header.
        telemetry_frame_header_t header;
        header.start_byte = byte;
        uint8_t *header_rest = (uint8_t *)&header + 1;
        const size_t header_rest_len = sizeof(header) - 1;

        if (uart_read_bytes(active_uart_num, header_rest, header_rest_len, pdMS_TO_TICKS(2)) != (int)header_rest_len) {
            continue;
        }

        // A length past the maximum means this 0xAA was payload data, not a real frame start.
        // Drop it and keep scanning - do NOT flush, the next start byte may already be in the buffer.
        if (header.payload_len > TELEMETRY_MAX_PAYLOAD_SIZE) {
            continue;
        }

        uint8_t payload[TELEMETRY_MAX_PAYLOAD_SIZE];
        if (header.payload_len > 0) {
            if (uart_read_bytes(active_uart_num, payload, header.payload_len, pdMS_TO_TICKS(2)) != (int)header.payload_len) {
                continue;
            }
        }

        telemetry_frame_tail_t tail;
        if (uart_read_bytes(active_uart_num, (uint8_t *)&tail, sizeof(tail), pdMS_TO_TICKS(2)) != (int)sizeof(tail)) {
            continue;
        }

        if (tail.end_byte != TELEMETRY_END_BYTE) {
            continue;
        }

        uint8_t crc_buffer[sizeof(header) + TELEMETRY_MAX_PAYLOAD_SIZE];
        memcpy(crc_buffer, &header, sizeof(header));
        if (header.payload_len > 0) {
            memcpy(crc_buffer + sizeof(header), payload, header.payload_len);
        }

        if (telemetry_calculate_crc16(crc_buffer, sizeof(header) + header.payload_len) != tail.crc16) {
            // Corrupted frame. We have consumed its bytes, so simply resume scanning from here.
            // Worst case we lose the one frame that follows it; the caller sees a gap, not a stall.
            continue;
        }

        out_message->header = header;
        if (header.payload_len > 0) {
            memcpy(out_message->payload, payload, header.payload_len);
        }
        out_message->tail = tail;
        return true;
    }

    return false;
}

esp_err_t uart_telemetry_send_message(telemetry_msg_type_t msg_type, const void *payload, uint8_t payload_len) {
    // Validate the lenght of the payload that needs to be sent
    if (payload_len > TELEMETRY_MAX_PAYLOAD_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    // Prepare the header for the telemetry message
    telemetry_frame_header_t header = {
        .start_byte = TELEMETRY_START_BYTE,
        .msg_type = (uint8_t)msg_type,
        .payload_len = payload_len,
        .seq = sequence_counter++,
    };

    // Prepare the tail for the telemetry message
    telemetry_frame_tail_t tail = {
        .crc16 = 0,
        .end_byte = TELEMETRY_END_BYTE,
    };

    // Compute the CRC for this message to be sent, based on the header and payload
    uint8_t crc_buffer[sizeof(header) + TELEMETRY_MAX_PAYLOAD_SIZE];
    memcpy(crc_buffer, &header, sizeof(header));
    if (payload_len > 0 && payload != NULL) {
        memcpy(crc_buffer + sizeof(header), payload, payload_len);
    }

    // Set the calculated CRC in the tail of the message
    tail.crc16 = telemetry_calculate_crc16(crc_buffer, sizeof(header) + payload_len);

    // Write the header to the UART interface
    int written = uart_write_bytes(active_uart_num, (const char *)&header, sizeof(header));
    if (written < 0) {
        return ESP_FAIL;
    }

    // If there is any payload, write it to the UART interface
    if (payload_len > 0 && payload != NULL) {
        written = uart_write_bytes(active_uart_num, (const char *)payload, payload_len);
        if (written < 0) {
            return ESP_FAIL;
        }
    }

    // Write the tail to the UART interface
    written = uart_write_bytes(active_uart_num, (const char *)&tail, sizeof(tail));
    return (written < 0) ? ESP_FAIL : ESP_OK;
}