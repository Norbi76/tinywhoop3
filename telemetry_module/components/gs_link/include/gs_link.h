#pragma once

#include "esp_err.h"
#include "telemetry_uart.h"

// gs_link.h - UDP bridge between the UART link and the Python ground station.
//
// WHAT THIS COMPONENT IS FOR
//   Tunnels the flight controller's PID tuning traffic over Wi-Fi. This module does NOT interpret
//   any of it - frames off the UART are batched into datagrams, datagrams coming back are split
//   into records and queued for the UART TX task. It is a pipe.
//
// HOW IT DOES ITS JOB
//   Downlink: gs_link_forward() APPENDS to a 1200-byte batch, gs_link_flush() sends it. One
//   datagram per RX drain rather than one per frame, because at 500 Hz the latter is 500
//   packets/second and the SoftAP chokes on packet rate long before bitrate.
//   Uplink:   the receive task QUEUES rather than sending, for the thread-safety reason spelled
//   out on gs_link_pop_uplink() below. Read that comment before changing anything here.
//   Peer:     latched from whoever last sent us a packet, which is what makes the link
//   self-healing across ground-station restarts and Wi-Fi drops.
//
// UDP bridge between the UART link and the Python ground station.
//
// Runs alongside the HTTP dashboard and is deliberately independent of it: the dashboard keeps
// getting its status frames exactly as before whether or not a ground station is connected, and
// the ground station works whether or not a browser is open.
//
// WIRE FORMAT - one datagram carries N concatenated records:
//
//     msg_type(1) | payload_len(1) | seq(2, little-endian) | payload(payload_len)
//
// No start byte, no CRC, no end byte. The UART frame needs all three because a serial line has
// no framing of its own and no error detection; UDP already provides both. Re-wrapping the UART
// frame would spend ~20% of the link on redundant bytes.
//
// PEER DISCOVERY: any inbound packet latches its sender as the current peer. That is what lets
// the ground station be restarted, or recover from a Wi-Fi drop, without power-cycling the
// drone. Until something has been received, outbound datagrams are dropped silently - there is
// nowhere to send them.

// UDP port the module listens on. The ground station sends here and receives on whatever
// ephemeral port it bound.
#define GS_LINK_PORT 14550

// Opens the socket and starts the receive task. Call after the Wi-Fi AP is up.
// @return ESP_OK on success, or an error code if the socket or the task could not be created.
esp_err_t gs_link_start(void);

// Appends one message to the outbound batch, flushing first if it would not fit.
// Does not transmit by itself. Safe to call for every frame received from the flight controller.
// @param msg Message to forward. Only the header and the first payload_len payload bytes are read.
void gs_link_forward(const telemetry_message_t *msg);

// Transmits and clears the batch. Call once per telemetry task iteration; batching is what keeps
// a 500 Hz sample stream from becoming 500 datagrams per second.
void gs_link_flush(void);

// Pops one message the ground station wants relayed to the flight controller.
//
// THE UPLINK IS A QUEUE, NOT A DIRECT SEND, for one reason: uart_telemetry_send_message() is not
// thread-safe. It bumps a shared sequence counter and then issues three separate
// uart_write_bytes() calls, so a second task calling it concurrently interleaves its bytes into
// another task's frame and produces CRC failures on the link that flies the drone. Queueing here
// and draining from uart_link's TX task keeps that task the single writer, exactly as the
// dashboard's gain updates already do.
//
// The queue lives in this component rather than in uart_link so the dependency runs one way -
// uart_link knows about gs_link, gs_link knows nothing about uart_link.
//
// @param msg_type Receives the message type.
// @param payload Destination buffer, which must be at least TELEMETRY_MAX_PAYLOAD_SIZE bytes.
// @param payload_len Receives the payload length.
// @return true if a message was popped, false if the queue is empty.
bool gs_link_pop_uplink(uint8_t *msg_type, void *payload, uint8_t *payload_len);
