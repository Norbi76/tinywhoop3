#pragma once

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "telemetry_uart.h"

esp_err_t web_page_init(void);
esp_err_t web_page_start_task(UBaseType_t core_id);
void web_page_store_message(const telemetry_message_t *message);
void web_page_get_control_frame(telemetry_control_payload_t *out);
