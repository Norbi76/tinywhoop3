#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "telemetry_uart.h"

// uart_link.h - the serial link to the flight controller. This is the component that flies the
// drone; Wi-Fi, the dashboard and the camera are all downstream of it.
//
// WHAT THIS COMPONENT IS FOR
//   Carries control frames out to the flight controller and status frames back, and relays the
//   ground station's tuning traffic in both directions. The wire protocol itself lives in
//   shared_components/telemetry_uart - this component owns the CADENCE and the task structure.
//
// HOW IT DOES ITS JOB
//   TX task, fixed 50 Hz: integrate the setpoints (control_state_update), send one control frame,
//   then piggyback a BOUNDED number of queued gain updates and ground-station uplink frames -
//   bounded so a burst of tuning traffic can never push the control frame late.
//   RX task, 5 ms wakeups: drain up to 64 frames with the resync reader, cache the status frame,
//   forward every frame to the ground station, then send ONE batched datagram.
//
// TX PRIORITY IS ABOVE RX, deliberately: a missed transmit slot risks the flight controller's
// 300 ms watchdog disarming the drone, while a late status frame only makes the dashboard stale.
//
// ONE WRITER RULE: uart_telemetry_send_message() is not thread-safe, so the TX task is the only
// task in this firmware that calls it. Everyone else queues - see uart_link_queue_gains() and
// gs_link_pop_uplink().
//
// UART link to the flight controller.
//
// Two tasks:
//   TX at a FIXED 50 Hz, regardless of whether the pilot is touching anything. The flight
//      controller disarms if it goes 300 ms without a valid frame, so a "send on change"
//      design would disarm the drone the moment the pilot stopped pressing buttons.
//      Queued gain updates piggyback on this task.
//   RX continuously, using the resynchronising reader so one corrupted byte costs a single
//      frame rather than the whole buffer.

// ---------------------------------------------------------------------------
// TODO(pins): CONFIRM AGAINST YOUR WIRING.
//
// >>> THESE ARE NOT THE PINS THE OLD main.c USED. <<<
// The previous code used TX=GPIO9, RX=GPIO8. On the XIAO ESP32-S3 Sense those two pins are
// the SD card's CMD and DATA0 lines on the expansion board (CLK=7, CMD=9, D0=8). Sharing them
// with the telemetry UART cannot work - whichever peripheral initialises second wins, and the
// symptom is either a dead link or an SD card that never mounts.
//
// GPIO 43/44 are the XIAO's D6/D7 pads (the chip's default UART0 TX/RX). They are free for
// UART1 use here as long as you are not depending on the USB-serial console on those pads;
// the XIAO's console runs over native USB, so this is normally fine.
// Check this against your actual harness before flashing.
// ---------------------------------------------------------------------------
#define UART_LINK_TX_PIN 43
#define UART_LINK_RX_PIN 44
#define UART_LINK_BAUD_RATE 460800
#define UART_LINK_PORT UART_NUM_1

// Fixed transmit rate. Must stay comfortably inside the flight controller's 300 ms watchdog.
#define UART_LINK_TX_HZ 50

// Initialises the UART driver.
// @return ESP_OK on success, or an error code on failure.
esp_err_t uart_link_init(void);

// Creates the TX and RX tasks.
// @param core_id Core to pin both tasks to. Should be core 0, leaving core 1 for the camera.
// @return ESP_OK on success, ESP_FAIL if either task could not be created.
esp_err_t uart_link_start(BaseType_t core_id);

// Queues a gain update for transmission on the next TX cycle.
// Returns immediately; the frame goes out within 20 ms.
// @param gains The gain payload to send.
// @return ESP_OK if queued, ESP_ERR_INVALID_ARG on NULL, ESP_ERR_NO_MEM if the queue is full.
esp_err_t uart_link_queue_gains(const telemetry_gains_payload_t *gains);

// Number of valid status frames received since boot, for diagnostics.
uint32_t uart_link_get_rx_count(void);

// Number of TX cycles completed since boot.
uint32_t uart_link_get_tx_count(void);
