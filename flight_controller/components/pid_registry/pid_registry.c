// pid_registry.c - the gain table, the debug queue and the test-signal generator.
//
// WHAT THIS FILE DOES
//   g_loops[]        pointers to flight_control's eight PID instances, filled by _bind().
//                    The registry does NOT own them; they are statics that outlive it.
//   apply/read_gains mutex-protected gain access for the telemetry task.
//   set_stream/      lock-free subscription and injection settings.
//   set_inject
//   inject_offset()  the four test waveforms. CALLED FROM fc_task - lock-free by contract.
//   publish()        samples one loop into the queue. CALLED FROM fc_task - drops, never blocks.
//   pop_debug()      the telemetry task's drain.
//
// HOW THE THREADING WORKS, AND WHY IT IS NOT UNIFORM
//   Three data flows, three different mechanisms, each picked for what the data is worth:
//     - Gains are multi-field and must be consistent -> mutex. Only telemetry-task callers ever
//       take it, so fc_task cannot be blocked by tuning traffic.
//     - Stream/inject settings are single aligned bytes and floats -> plain volatile. A torn read
//       costs one extra, missing or distorted sample of an operator-triggered test. Not worth a
//       lock in the 1 kHz path.
//     - Debug samples are a stream -> queue, with a ZERO-TIMEOUT send. The drop is the design.
//       A blocking send here would stall fc_task behind the telemetry task, which is precisely
//       the failure this whole component is arranged to prevent.
//
// EVERY FAILURE HERE IS SOFT
//   If the queue or the mutex cannot be created, the corresponding feature is disabled and logged
//   and the flight loop is untouched. Instrumentation must never be able to ground the aircraft.

#include "pid_registry.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "PID_REG";

// 64 samples of 30 bytes is under 2 KB and covers ~128 ms at 500 Hz, which is far more slack than
// the telemetry task's 20 ms cycle needs. Deeper would only buy the ability to fall further
// behind before dropping, which is not a useful thing to buy.
#define PID_DEBUG_QUEUE_DEPTH 64

static pid_controller_t *g_loops[PID_LOOP_COUNT];
static QueueHandle_t g_debug_queue;
static SemaphoreHandle_t g_gains_mutex;

// Stream selection. Read from fc_task without a lock: both are single aligned bytes, so a
// concurrent write from the telemetry task can only ever be seen as the old or the new value,
// never as a mixture. The worst case is one extra or one missing sample around a retune.
static volatile uint8_t g_stream_loop = PID_LOOP_ALL;
static volatile uint8_t g_stream_divider = 1;
static uint8_t g_decimator[PID_LOOP_COUNT];

// Test-signal injection. Same lock-free reasoning: fc_task reads these, the telemetry task
// writes them, and a torn read costs at most one distorted sample of an operator-triggered
// test signal.
static volatile uint8_t g_inject_loop = PID_LOOP_ALL;   // PID_LOOP_ALL == "no loop selected"
static volatile uint8_t g_inject_mode;
static volatile float g_inject_amplitude;
static volatile float g_inject_period_s;
static volatile float g_inject_start_s;

static uint32_t g_drop_count;

void pid_registry_init(void) {
    memset(g_loops, 0, sizeof(g_loops));
    memset(g_decimator, 0, sizeof(g_decimator));

    g_debug_queue = xQueueCreate(PID_DEBUG_QUEUE_DEPTH, sizeof(telemetry_pid_debug_payload_t));
    if (g_debug_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create the PID debug queue - streaming disabled");
    }

    g_gains_mutex = xSemaphoreCreateMutex();
    if (g_gains_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create the gains mutex - live tuning disabled");
    }

    // Nothing streams until the ground station asks for something. Starting subscribed to every
    // loop at 1 kHz would fill the queue before anyone was listening.
    g_stream_loop = PID_LOOP_ALL;
    g_stream_divider = 0;
    g_inject_loop = PID_LOOP_ALL;
    g_inject_mode = 0;
    g_drop_count = 0;
}

void pid_registry_bind(pid_loop_id_t id, pid_controller_t *pid) {
    if ((int)id < 0 || (int)id >= PID_LOOP_COUNT || pid == NULL) {
        ESP_LOGE(TAG, "Refusing to bind loop %d", (int)id);
        return;
    }

    g_loops[id] = pid;
}

void pid_registry_apply_gains(const telemetry_pid_gains_payload_t *g) {
    if (g == NULL || g->loop_id >= PID_LOOP_COUNT) {
        return;
    }

    pid_controller_t *pid = g_loops[g->loop_id];
    if (pid == NULL) {
        ESP_LOGW(TAG, "Gains for unbound loop %u ignored", g->loop_id);
        return;
    }

    if (g_gains_mutex != NULL && xSemaphoreTake(g_gains_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGW(TAG, "Gains mutex busy, dropping update for loop %u", g->loop_id);
        return;
    }

    pid->kp = g->kp;
    pid->ki = g->ki;
    pid->kd = g->kd;

    // THE THREE LIMITS TREAT ZERO AS "LEAVE THIS ALONE", and that is a safety property, not a
    // convenience. A zero out_limit would make clampf(x, 0, 0) return 0 for every input, so the
    // loop would output nothing at all for the rest of the flight - on a rate loop that is the
    // axis gone. Nothing legitimately asks for a zero limit, but plenty of things produce one by
    // omission: a gain profile carrying only kp/ki/kd, or an apply that happens before a
    // read-back has filled the limit fields in.
    //
    // This does not collide with the all-six-zero read overload, which is handled before we get
    // here: a payload with real gains and zero limits is a genuine partial set.
    if (g->i_limit > 0.0f) {
        pid->integrator_limit = g->i_limit;
    }

    // The payload carries one symmetric limit while the controller keeps an independent min and
    // max. Every instance the cascade creates is symmetric today, so nothing is lost - but an
    // asymmetric loop added later would be silently symmetrised here.
    if (g->out_limit > 0.0f) {
        pid->out_max = g->out_limit;
        pid->out_min = -pid->out_max;
    }

    // The filter coefficient depends on the loop period as well as the cutoff, which is why
    // pid_controller_t retains nominal_dt. A zero or negative cutoff would divide by zero, so it
    // leaves the existing filter alone rather than producing a NaN inside the 1 kHz loop.
    if (g->d_cutoff_hz > 0.0f && pid->nominal_dt > 0.0f) {
        const float rc = 1.0f / (2.0f * (float)M_PI * g->d_cutoff_hz);
        pid->d_alpha = pid->nominal_dt / (rc + pid->nominal_dt);
        pid->d_cutoff_hz = g->d_cutoff_hz;
    }

    // See the header: this is the whole reason apply_gains is not just pid_set_gains.
    pid->integrator = 0.0f;

    if (g_gains_mutex != NULL) {
        xSemaphoreGive(g_gains_mutex);
    }
}

void pid_registry_read_gains(pid_loop_id_t id, telemetry_pid_gains_payload_t *out) {
    if (out == NULL) {
        return;
    }

    memset(out, 0, sizeof(*out));
    if ((int)id < 0 || (int)id >= PID_LOOP_COUNT) {
        return;
    }

    out->loop_id = (uint8_t)id;

    const pid_controller_t *pid = g_loops[id];
    if (pid == NULL) {
        return;
    }

    if (g_gains_mutex != NULL && xSemaphoreTake(g_gains_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }

    out->kp = pid->kp;
    out->ki = pid->ki;
    out->kd = pid->kd;
    out->i_limit = pid->integrator_limit;
    out->out_limit = pid->out_max;
    out->d_cutoff_hz = pid->d_cutoff_hz;

    if (g_gains_mutex != NULL) {
        xSemaphoreGive(g_gains_mutex);
    }
}

void pid_registry_set_stream(uint8_t loop_id, uint8_t divider) {
    if (loop_id != PID_LOOP_ALL && loop_id >= PID_LOOP_COUNT) {
        ESP_LOGW(TAG, "Ignoring stream select for loop %u", loop_id);
        return;
    }

    memset(g_decimator, 0, sizeof(g_decimator));

    // Order matters: set the divider before the loop id, so the selection never goes live for an
    // instant with the previous divider still in place.
    g_stream_divider = (divider == 0) ? 1 : divider;
    g_stream_loop = loop_id;
}

void pid_registry_set_inject(const telemetry_pid_inject_payload_t *inj) {
    if (inj == NULL) {
        return;
    }

    if (inj->mode == 0 || inj->loop_id >= PID_LOOP_COUNT) {
        g_inject_mode = 0;
        g_inject_loop = PID_LOOP_ALL;
        return;
    }

    // t=0 is latched here rather than passed in, so the ground station does not have to know
    // anything about the flight controller's clock.
    g_inject_start_s = (float)((double)esp_timer_get_time() / 1000000.0);
    g_inject_amplitude = inj->amplitude;
    g_inject_period_s = (inj->period_s > 0.0f) ? inj->period_s : 1.0f;
    g_inject_mode = inj->mode;
    g_inject_loop = inj->loop_id;
}

float pid_registry_inject_offset(pid_loop_id_t id, float t_s) {
    const uint8_t mode = g_inject_mode;
    if (mode == 0 || g_inject_loop != (uint8_t)id) {
        return 0.0f;
    }

    const float amplitude = g_inject_amplitude;
    const float period = g_inject_period_s;
    const float elapsed = t_s - g_inject_start_s;

    if (elapsed < 0.0f || period <= 0.0f) {
        return 0.0f;
    }

    switch (mode) {
        case 1:
            // Step: constant from the trigger onwards. The plot's overshoot and settling-time
            // readouts are computed off this one.
            return amplitude;

        case 2:
            // Doublet: +A, then -A, then nothing. Excites the loop in both directions and leaves
            // the aircraft where it started, which is why it is the safe one to use in the air.
            if (elapsed < period * 0.5f) {
                return amplitude;
            }
            if (elapsed < period) {
                return -amplitude;
            }
            return 0.0f;

        case 3: {
            // Square: continuous alternation, for watching the loop settle repeatedly.
            const float phase = fmodf(elapsed, period);
            return (phase < period * 0.5f) ? amplitude : -amplitude;
        }

        default:
            return 0.0f;
    }
}

void pid_registry_publish(pid_loop_id_t id, const pid_controller_t *pid) {
    if (pid == NULL || (int)id < 0 || (int)id >= PID_LOOP_COUNT || g_debug_queue == NULL) {
        return;
    }

    // Cheapest test first: on an unsubscribed loop this is the whole cost of the call, which is
    // what makes it acceptable to leave in the 1 kHz path permanently.
    const uint8_t selected = g_stream_loop;
    if (g_stream_divider == 0) {
        return;
    }
    if (selected != PID_LOOP_ALL && selected != (uint8_t)id) {
        return;
    }

    const uint8_t divider = g_stream_divider;
    if (++g_decimator[id] < divider) {
        return;
    }
    g_decimator[id] = 0;

    telemetry_pid_debug_payload_t sample = {
        .t_us = (uint32_t)esp_timer_get_time(),   // truncated on purpose, see the payload comment
        .loop_id = (uint8_t)id,
        .flags = 0,
        .setpoint = pid->last_setpoint,
        .measurement = pid->last_measurement,
        .p_term = pid->last_p,
        .i_term = pid->last_i,
        .d_term = pid->last_d,
        .output = pid->last_output,
    };

    if (pid->integrator_clamped) sample.flags |= PID_FLAG_I_CLAMPED;
    if (pid->out_saturated)      sample.flags |= PID_FLAG_OUT_SAT;
    // PID_FLAG_MEAS_STALE has no producer on this airframe: nothing tracks per-loop measurement
    // age, and the loops that could go stale (altitude, velocity) simply stop running when their
    // nav validity flag drops rather than running on an old sample.

    // Zero timeout, and the drop is the point. A blocking send here would stall fc_task behind
    // the telemetry task, which is the one thing this whole component exists to prevent.
    if (xQueueSend(g_debug_queue, &sample, 0) != pdTRUE) {
        g_drop_count++;
    }
}

bool pid_registry_pop_debug(telemetry_pid_debug_payload_t *out, TickType_t wait) {
    if (out == NULL || g_debug_queue == NULL) {
        return false;
    }

    return xQueueReceive(g_debug_queue, out, wait) == pdTRUE;
}

uint32_t pid_registry_get_drop_count(void) {
    return g_drop_count;
}
