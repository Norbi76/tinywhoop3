#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "pid_controller.h"
#include "telemetry_uart.h"

// pid_registry.h - live PID tuning and instrumentation, the bridge to the ground station.
//
// WHAT THIS COMPONENT IS FOR
//   Lets the Python ground station read and write PID gains, watch a loop's internals, and inject
//   a test signal into a setpoint - all in flight, without reflashing. It exists as a separate
//   component so flight_control does not have to know about telemetry and telemetry does not have
//   to know about the cascade's internals.
//
// HOW IT DOES IT - three different synchronisation strategies, chosen per data flow
//   gains      : mutex (telemetry-task writers only; fc_task never takes it)
//   stream/inject settings : lock-free volatile bytes (a torn read costs one distorted sample)
//   debug samples : a FreeRTOS queue with zero-timeout sends that DROP when full
//   Each choice is justified where it is implemented in pid_registry.c.
//
// ADDRESSING
//   This component uses pid_loop_id_t (outer-to-inner, the ground station's numbering), NOT the
//   web dashboard's telemetry_loop_id_t. pid_registry_bind() is what reconciles the two - see
//   README.md and shared_components/telemetry_uart/README.md.
//
// Live PID tuning and instrumentation registry.
//
// Sits between the control cascade and the telemetry link so that neither has to know about the
// other. flight_control binds its 8 pid_controller_t instances here once at init; from then on
// the registry can read and write their gains, sample their internals, and hand the control loop
// a test-signal offset to add to a setpoint.
//
// THE ONE RULE THIS COMPONENT EXISTS TO ENFORCE: everything called from fc_task is non-blocking.
// pid_registry_publish() and pid_registry_inject_offset() run inside the 1 kHz control task and
// never take a lock, never touch the UART and never log. Publishing is a zero-timeout queue send
// that drops the sample if the queue is full, because a dropped plot point costs nothing and a
// late control iteration costs the drone.
//
// The mutex protects the gain tables against concurrent writers on the telemetry task only.
// fc_task never takes it.

// Prepares the debug queue and the mutex. Call once, before flight_control_init() binds anything.
void pid_registry_init(void);

// Associates a loop id with a live PID instance. Call once per loop during flight controller init.
// The registry does not own the controller and never frees it.
// @param id Ground-station loop id.
// @param pid Pointer to the controller, which must outlive the registry (they are all statics).
void pid_registry_bind(pid_loop_id_t id, pid_controller_t *pid);

// Writes a full gain set to one loop and ZEROES ITS INTEGRATOR.
//
// The reset is not incidental. The integrator holds the ki-scaled accumulation, so raising ki
// mid-flight leaves whatever was accumulated under the old gain sitting in the output as a step.
// On a 50 g quad with an altitude loop that step is the ceiling. This is the deliberate
// difference from pid_set_gains(), which preserves the integrator on purpose for the dashboard's
// small-nudge tuning - see the comment on pid_controller_t.integrator.
//
// The three LIMIT fields (i_limit, out_limit, d_cutoff_hz) treat zero as "leave this one alone".
// kp/ki/kd do not - a zero gain is a meaningful thing to ask for. This asymmetry exists because a
// zero out_limit would clamp the loop's output to zero for the rest of the flight, and the
// commonest way to send one is by omission rather than by intent.
//
// @param g Gain set. A message whose six float fields are all exactly zero is a read request and
//          must be answered with pid_registry_read_gains() instead of passed here.
void pid_registry_apply_gains(const telemetry_pid_gains_payload_t *g);

// Reads one loop's live gains back into a payload ready to send.
// @param id Loop to read.
// @param out Filled in on success; zeroed and left with loop_id set if the loop is not bound.
void pid_registry_read_gains(pid_loop_id_t id, telemetry_pid_gains_payload_t *out);

// Subscribes the debug stream to one loop, replacing any previous selection.
// @param loop_id pid_loop_id_t, or PID_LOOP_ALL for every loop at once.
// @param divider Emit one sample per N ticks of that loop. 0 is treated as 1.
void pid_registry_set_stream(uint8_t loop_id, uint8_t divider);

// Arms, retargets or cancels the test-signal injection. Only one loop is ever injected at a time,
// so this replaces rather than adds. Latches the current time as the signal's t=0.
// @param inj Injection settings; mode 0 cancels.
void pid_registry_set_inject(const telemetry_pid_inject_payload_t *inj);

// Returns the additive setpoint perturbation for one loop, or 0 if that loop is not the injection
// target. Called from fc_task - lock-free, no logging.
// @param id Loop about to be updated.
// @param t_s Current time in seconds, on the same timebase as esp_timer_get_time().
// @return Offset to ADD to the loop's setpoint before calling pid_update().
float pid_registry_inject_offset(pid_loop_id_t id, float t_s);

// Samples one loop's internals into the debug queue. Call at the end of every pid_update() for
// that loop, from inside the control task.
//
// Returns immediately if the loop is not subscribed or if the decimation counter has not come
// round. On a full queue the sample is DROPPED and the drop counter incremented - that is the
// correct behaviour here, not something to fix by blocking.
//
// @param id Loop that was just updated.
// @param pid The controller, read straight after its pid_update() call.
void pid_registry_publish(pid_loop_id_t id, const pid_controller_t *pid);

// Pops one queued debug sample, for the telemetry task to send.
// @param out Destination sample.
// @param wait Ticks to block. Pass 0 from anything latency-sensitive.
// @return true if a sample was written to out.
bool pid_registry_pop_debug(telemetry_pid_debug_payload_t *out, TickType_t wait);

// Total debug samples dropped on a full queue since boot. A non-zero and rising value means the
// telemetry task is not draining fast enough for the selected divider, so the plot has gaps.
// @return The running drop count.
uint32_t pid_registry_get_drop_count(void);
