// main.c - the flight controller's entry point: startup ordering and the three FreeRTOS tasks.
//
// WHAT THIS FILE DOES
//   No control logic of its own. It creates the tasks, brings the components up in an order that
//   is dictated by hardware dependencies, and owns the cross-task plumbing.
//
//     fc_task         core 1, top priority, 1000 Hz - IMU -> attitude -> nav snapshot -> cascade
//     sensor_task     core 0, prio 6,        100 Hz - lives in sensor_task.c
//     telemetry_task  core 0, prio 5,         50 Hz - status/debug/IMU out, control+tuning in
//
//   The split is by TIMING SENSITIVITY, not by subject. fc_task gets core 1 (APP_CPU) because
//   core 0 (PRO_CPU) runs Wi-Fi, lwIP and IDF background work; the two slow tasks go on core 0
//   precisely because their I/O blocks.
//
// THE ONE RULE THAT SHAPES EVERYTHING HERE
//   The 1 kHz side never waits. Every cross-task read in fc_task uses a ZERO timeout and carries
//   on with its previous copy on failure. Stale data by one cycle is always preferable to a
//   missed 1 ms deadline.
//
// STARTUP ORDER IS NOT ARBITRARY - see the comments in app_main(). Two constraints drive it:
//   1. Motors must be configured and driven to zero before any code can command thrust.
//   2. The ToF sensor attaches to the I2C bus that imu_setup() CREATES, and imu_setup() runs
//      inside fc_task - hence imu_ready_semaphore, which app_main blocks on before starting the
//      navigation sensors.
//

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "imu_driver.h"
#include "esp_timer.h"
#include "telemetry_uart.h"

#include "attitude_estimator.h"
#include "nav_estimator.h"
#include "motor_driver.h"
#include "battery_monitor.h"
#include "flight_control.h"
#include "pid_registry.h"
#include "sensor_task.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "FC_MAIN";
// #define ALPHA 0.70

// How many PID debug samples the telemetry task forwards per 20 ms cycle. 32 covers 1600
// samples/second, comfortably above the 500 Hz the ground station asks for at its fastest
// setting, so the queue only backs up if the UART itself is the bottleneck.
#define PID_DEBUG_PER_CYCLE 32

// ===========================================================================
// MOTOR BENCH MODE - measuring MOTOR_IDLE_THRUST
//
// Set MOTOR_BENCH_MODE to 1 and app_main() brings up the motor driver and NOTHING ELSE: no
// IMU, no estimators, no flight control, no UART, no arming gate. Set it back to 0 before
// flying. Left on by accident the firmware does not fly badly - it does not fly at all, which
// is the failure mode you want from a switch like this.
//
// WHAT IT DOES
//   Holds the motors at exactly BENCH_THRUST, cycling BENCH_ON_MS on and BENCH_OFF_MS off,
//   forever. Nothing ramps and nothing changes on its own: the only number that reaches the
//   motors is the one written below. To test another value, edit it, reflash, watch again.
//
// WHY IT CYCLES INSTEAD OF JUST HOLDING
//   The quantity being measured is "does this motor start RELIABLY from a dead stop", and one
//   successful start does not answer that. Every on-transition is a fresh start attempt from
//   rest, so watching ten cycles gives ten trials. A motor that catches on eight of ten is
//   telling you this thrust is below its real floor, which a single hold would hide completely.
//
// WHAT TO LOOK FOR
//   All four motors starting on EVERY cycle, promptly, without a stutter or a nudge. A motor
//   that hesitates, or starts on some cycles and not others, has not met the bar. The lowest
//   value at which all four pass every cycle is the answer.
//
// PROPS STAY ON. Against the usual bench rule, deliberately: a coreless brushed motor needs
//   materially more torque to start under a prop's load than bare, so a bare-shaft measurement
//   returns a number that is too low - the same direction of error already in the code.
//   RESTRAIN THE AIRFRAME and keep hands and face clear. BENCH_THRUST is never near hover, but
//   restraint is still the only safe way to run this.
//
// SUGGESTED SEQUENCE (bisection - about four flashes rather than eight)
//   0.12 is known to work: all four already start at it, since that is what the mixer floors
//   them to today. So start below it and bisect.
//
//     0.06  -> all four start every cycle?  no  -> try 0.09
//     0.09  -> all four start every cycle?  yes -> try 0.075
//     0.075 -> ...                          no  -> try 0.08
//     0.08  -> ...                          yes -> answer is 0.08
//
//   Then add a little margin for a tired pack and a cold motor: round UP, and prefer the value
//   that still worked to the one that just barely did.
//
// PACK VOLTAGE MATTERS
//   Start duty depends on pack voltage, so a value that passes on a full pack can fail on a
//   half-empty one. Take the measurement on a freshly charged pack, and if a value is
//   borderline, re-check it near the end of a pack before trusting it.
// ===========================================================================
#define MOTOR_BENCH_MODE 1

// The thrust command under test. This is the ONLY value that reaches the motors.
#define BENCH_THRUST 0.06f

// Which motor: 0-3 for one on its own, or -1 for all four together.
// All four together is usually what you want - they then see the same pack voltage at the same
// instant, so a motor that needs more than its siblings shows up directly as the odd one out.
#define BENCH_MOTOR (-1)

// On and off times for one trial. 2.5 s on is long enough to see and hear clearly; 2 s off is
// long enough for a coreless motor to come fully to rest, which is what makes the next
// on-transition a genuine start from dead stop rather than a re-acceleration.
#define BENCH_ON_MS  2500
#define BENCH_OFF_MS 2000

static SemaphoreHandle_t imu_data_mutex;
static telemetry_imu_payload_t latest_imu_payload;

// Signalled by fc_task once the IMU is up and calibrated. app_main waits on this before
// initialising the navigation sensors, because the VL53L1X shares the I2C bus that
// imu_setup() creates and must not touch it first.
static SemaphoreHandle_t imu_ready_semaphore;

// THE 1 kHz CONTROL TASK. Core 1, highest priority, and the only writer of the motors.
//
// Startup half: owns the IMU end to end (setup + both calibration sweeps), then signals
// imu_ready_semaphore so app_main can bring up the sensors that share the I2C bus it just made.
//
// Loop half, per 1 ms tick:
//   read IMU -> convert -> compute dt -> publish the IMU telemetry snapshot (zero-timeout mutex)
//   -> attitude_update/get -> grab the latest nav state (zero-timeout mutex) -> run the cascade.
//
// Nothing slow is permitted past the startup half: no logging in the steady-state path, no
// blocking mutex take, no I/O other than the single IMU burst read.
void fc_task(void *args) { //semnatura unui task freeRTOS trebuie sa contina un param de tip void*
    ESP_LOGI(TAG, "Flight controll task started on core %d", xPortGetCoreID());
    //init imu, pwm
    esp_err_t error = imu_setup();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "IMU setup failed: %s", esp_err_to_name(error));
        vTaskDelete(NULL);
        return;
    }
    // float roll_offset = 0.0f, pitch_offset = 0.0f;
    imu_calibrate_acc();
    imu_calibrate_gyro();

    // The IMU is up and the I2C bus exists - app_main can now bring up the ToF and the flow
    // sensor without racing us for the bus.
    if (imu_ready_semaphore != NULL) {
        xSemaphoreGive(imu_ready_semaphore);
    }

    // 1 tick == 1 ms because CONFIG_FREERTOS_HZ is 1000 (see sdkconfig.defaults).
    // NOTE: this used to be pdMS_TO_TICKS(1000), which is 1000 ms, so the "1 kHz" loop was
    // actually running at 1 Hz. Raising the tick rate is what makes 1 kHz reachable at all.
    const TickType_t xFreq = pdMS_TO_TICKS(1); // 1ms -> 1kHz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    int64_t last_time = esp_timer_get_time();


    imu_raw_data_t imu_raw_data;
    imu_physical_data_t imu_physical_data;

    float roll_angle = 0.0f, pitch_angle = 0.0f;
    // float roll_angle_comp = 0.0f, pitch_angle_comp = 0.0f;

    attitude_state_t attitude;
    nav_state_t nav_state = {0};   // retained between iterations if the mutex is busy

    for (;;) {
        error = imu_read_raw_data(&imu_raw_data);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "IMU read failed: %s", esp_err_to_name(error));
            // A dead IMU means no attitude, which means the drone must not be flying.
            motor_all_stop();
            vTaskDelay(pdMS_TO_TICKS(10)); // wait a bit before retrying
            continue;
        }
        imu_convert_raw_to_physical(&imu_raw_data, &imu_physical_data);


        const int64_t current_time = esp_timer_get_time();
        float dt = (float)(current_time - last_time) / 1000000.0f; // convert to seconds
        last_time = current_time;

        // Guard against a silly dt after a scheduling hiccup - a huge dt would produce a huge
        // derivative and integrator step in the cascade.
        if (dt <= 0.0f || dt > 0.02f) {
            dt = 1.0f / 1000.0f;
        }

        imu_compute_roll_pitch(imu_physical_data.acc_x_g, imu_physical_data.acc_y_g, imu_physical_data.acc_z_g, &roll_angle, &pitch_angle);

        if (imu_data_mutex != NULL && xSemaphoreTake(imu_data_mutex, 0) == pdTRUE) {
            latest_imu_payload.acc_x = imu_physical_data.acc_x_g;
            latest_imu_payload.acc_y = imu_physical_data.acc_y_g;
            latest_imu_payload.acc_z = imu_physical_data.acc_z_g;
            latest_imu_payload.gyro_x = imu_physical_data.gyro_x_dps;
            latest_imu_payload.gyro_y = imu_physical_data.gyro_y_dps;
            latest_imu_payload.gyro_z = imu_physical_data.gyro_z_dps;
            latest_imu_payload.roll = roll_angle;
            latest_imu_payload.pitch = pitch_angle;
            latest_imu_payload.temperature = imu_physical_data.temp_C;
            xSemaphoreGive(imu_data_mutex);
        }
        // roll_angle_comp = ALPHA * (roll_angle_comp + imu_physical_data.gyro_x_dps * dt) + (1 - ALPHA) * roll_angle;
        // pitch_angle_comp = ALPHA * (pitch_angle_comp + imu_physical_data.gyro_y_dps * dt) + (1 - ALPHA) * pitch_angle;

        // 1. Fuse accelerometer and gyro into an attitude estimate.
        attitude_update(&imu_physical_data, dt);
        attitude_get(&attitude);

        // 2. Pick up the latest navigation state from sensor_task. Zero-timeout: if the mutex
        //    is busy we keep last iteration's copy rather than blocking the 1 kHz loop.
        sensor_task_get_nav_state(&nav_state);

        // 3. Run the cascade, the arming state machine and the mixer. This writes the motors,
        //    including forcing them to zero on every disarmed path.
        flight_control_update(&attitude, &nav_state, dt);

        // Așteaptă exact până la următoarea milisecundă
        vTaskDelayUntil(&xLastWakeTime, xFreq);
    }
}

// THE 50 Hz LINK TASK. Core 0. Everything that talks to the telemetry module happens here.
//
// TRANSMIT ORDER IS DELIBERATE: status frame, then PID debug, then (every 5th cycle) the raw IMU
// frame. The status frame is what the dashboard and the pilot depend on, so it goes out first no
// matter how much tuning traffic is queued behind it.
//
// RECEIVE DRAINS THE WHOLE BUFFER rather than taking one frame per cycle - otherwise RX backs up
// and the control frames actually acted on get progressively staler. It uses the RESYNC reader
// so one corrupted byte costs a single frame instead of a full flush, because a flush would take
// good control frames with it and trip flight_control's 300 ms link watchdog.
//
// Every inbound handler length-checks payload_len against its struct before casting. That check
// is the only thing between a truncated frame and a read past the end of the payload buffer.
void telemetry_task(void *args) {
    //init uart...
    ESP_LOGI(TAG, "Telemtry transmission task started on core %d", xPortGetCoreID());

    telemetry_message_t rx_message;
    uint32_t cycle = 0;

    for (;;) {
        // --- Status frame, every cycle (50 Hz) -----------------------------
        // This is what the dashboard reads back: attitude, nav, motors, arming and the
        // validity flags.
        telemetry_status_payload_t status;
        flight_control_get_status(&status);

        esp_err_t send_error = uart_telemetry_send_message(
            TELEMETRY_MSG_STATUS,
            &status,
            sizeof(status)
        );
        if (send_error != ESP_OK) {
            ESP_LOGE(TAG, "Status telemetry send failed: %s", esp_err_to_name(send_error));
        }

        // --- PID debug samples ---------------------------------------------
        // Sent after the status frame and before the IMU frame: the status frame is what the
        // dashboard and the link watchdog depend on, so it goes out first regardless of how much
        // tuning traffic is queued behind it.
        //
        // Bounded drain. Nothing here blocks on the queue (zero wait) - an empty queue ends the
        // loop immediately, which is the normal case when no loop is subscribed.
        telemetry_pid_debug_payload_t pid_sample;
        for (int i = 0; i < PID_DEBUG_PER_CYCLE; i++) {
            if (!pid_registry_pop_debug(&pid_sample, 0)) {
                break;
            }

            // Deliberately unlogged even on failure: this runs up to 32 times per cycle and a
            // per-sample log line would saturate the console UART on its own.
            uart_telemetry_send_message(TELEMETRY_MSG_PID_DEBUG, &pid_sample, sizeof(pid_sample));
        }

        // --- Raw IMU frame, every 5th cycle (10 Hz) ------------------------
        // Not needed for control; kept for logging and for the raw-data view.
        if ((cycle % 5) == 0) {
            telemetry_imu_payload_t imu_payload = {0};

            if (imu_data_mutex != NULL && xSemaphoreTake(imu_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                imu_payload = latest_imu_payload;
                xSemaphoreGive(imu_data_mutex);
            }

            send_error = uart_telemetry_send_message(
                TELEMETRY_MSG_IMU,
                &imu_payload,
                sizeof(imu_payload)
            );
            if (send_error != ESP_OK) {
                ESP_LOGE(TAG, "IMU telemetry send failed: %s", esp_err_to_name(send_error));
            }
        }

        // --- Receive ---------------------------------------------------------
        // Drain everything queued rather than taking one frame per 20 ms cycle, otherwise the
        // RX buffer backs up and the control frames we act on get progressively staler.
        // The resync reader is used here: it discards one bad frame instead of flushing the
        // whole buffer, which matters because a flush would drop good control frames and
        // trip the 300 ms link watchdog.
        while (uart_telemetry_read_message_resync(&rx_message, 2)) {
            switch ((telemetry_msg_type_t)rx_message.header.msg_type) {
                case TELEMETRY_MSG_CONTROL: {
                    if (rx_message.header.payload_len < sizeof(telemetry_control_payload_t)) {
                        ESP_LOGW(TAG, "Short control frame: %u bytes", rx_message.header.payload_len);
                        break;
                    }
                    const telemetry_control_payload_t *control =
                        (const telemetry_control_payload_t *)rx_message.payload;

                    // Feeds the controller AND pets the link watchdog.
                    flight_control_set_control_input(control);
                    break;
                }
                case TELEMETRY_MSG_GAINS: {
                    if (rx_message.header.payload_len < sizeof(telemetry_gains_payload_t)) {
                        ESP_LOGW(TAG, "Short gains frame: %u bytes", rx_message.header.payload_len);
                        break;
                    }
                    const telemetry_gains_payload_t *gains =
                        (const telemetry_gains_payload_t *)rx_message.payload;

                    esp_err_t gain_error = flight_control_set_gains(
                        (telemetry_loop_id_t)gains->loop_id, gains->kp, gains->ki, gains->kd);
                    if (gain_error != ESP_OK) {
                        ESP_LOGW(TAG, "Rejected gains for loop %u", gains->loop_id);
                    }
                    break;
                }
                case TELEMETRY_MSG_PID_GAINS: {
                    if (rx_message.header.payload_len < sizeof(telemetry_pid_gains_payload_t)) {
                        ESP_LOGW(TAG, "Short PID gains frame: %u bytes", rx_message.header.payload_len);
                        break;
                    }
                    const telemetry_pid_gains_payload_t *gains =
                        (const telemetry_pid_gains_payload_t *)rx_message.payload;

                    // All six floats exactly zero means "tell me what you have", not "set
                    // everything to zero" - an all-zero gain set would disable the loop and is
                    // never something anyone means. See telemetry_uart.h.
                    const bool is_read_request = (gains->kp == 0.0f && gains->ki == 0.0f &&
                                                  gains->kd == 0.0f && gains->i_limit == 0.0f &&
                                                  gains->out_limit == 0.0f &&
                                                  gains->d_cutoff_hz == 0.0f);

                    if (is_read_request) {
                        telemetry_pid_gains_payload_t live;
                        pid_registry_read_gains((pid_loop_id_t)gains->loop_id, &live);
                        uart_telemetry_send_message(TELEMETRY_MSG_PID_GAINS, &live, sizeof(live));
                    } else {
                        pid_registry_apply_gains(gains);
                    }
                    break;
                }
                case TELEMETRY_MSG_PID_SELECT: {
                    if (rx_message.header.payload_len < sizeof(telemetry_pid_select_payload_t)) {
                        ESP_LOGW(TAG, "Short PID select frame: %u bytes", rx_message.header.payload_len);
                        break;
                    }
                    const telemetry_pid_select_payload_t *select =
                        (const telemetry_pid_select_payload_t *)rx_message.payload;

                    pid_registry_set_stream(select->loop_id, select->divider);
                    break;
                }
                case TELEMETRY_MSG_PID_INJECT: {
                    if (rx_message.header.payload_len < sizeof(telemetry_pid_inject_payload_t)) {
                        ESP_LOGW(TAG, "Short PID inject frame: %u bytes", rx_message.header.payload_len);
                        break;
                    }
                    const telemetry_pid_inject_payload_t *inject =
                        (const telemetry_pid_inject_payload_t *)rx_message.payload;

                    pid_registry_set_inject(inject);
                    break;
                }
                default:
                    // Other message types are not expected on this direction of the link.
                    break;
            }
        }

        cycle++;
        vTaskDelay(pdMS_TO_TICKS(20));
    }

}

// Brings the system up in a fixed, dependency-driven order and then hands off to the tasks.
//
// The order below is load-bearing; the comment at each step says why that step is where it is.
// Note the asymmetry in failure handling: anything that could let the motors run unsupervised
#if MOTOR_BENCH_MODE
// ---------------------------------------------------------------------------
// Writes BENCH_THRUST to whichever motors are under test, or zero to stop them. Routed through
// motor_set_thrust_all() rather than four separate writes so that all four channels update in
// one pass - when comparing motors against each other, a stagger of even a few milliseconds
// between them is one more thing to have to think about.
// ---------------------------------------------------------------------------
static void bench_drive(bool on)
{
    float thrust[MOTOR_COUNT] = {0};

    if (on) {
        for (int i = 0; i < MOTOR_COUNT; i++) {
            if (BENCH_MOTOR < 0 || i == BENCH_MOTOR) {
                thrust[i] = BENCH_THRUST;
            }
        }
    }

    motor_set_thrust_all(thrust);
}

// ---------------------------------------------------------------------------
// The session: a ten-second pause so you can step back, then on/off cycles at BENCH_THRUST
// until the board is powered down. The cycle number is logged if a serial cable happens to be
// attached, but nothing here depends on that - the whole test is designed to be read by
// watching the motors.
// ---------------------------------------------------------------------------
static void motor_bench_run(void)
{
    ESP_LOGW(TAG, "=== MOTOR BENCH MODE - this firmware does NOT fly ===");
    ESP_LOGW(TAG, "Props ON, airframe RESTRAINED, hands and face clear.");

    if (BENCH_MOTOR < 0) {
        ESP_LOGI(TAG, "all four motors at thrust %.3f", (double)BENCH_THRUST);
    } else {
        ESP_LOGI(TAG, "motor M%d at thrust %.3f", BENCH_MOTOR, (double)BENCH_THRUST);
    }
    ESP_LOGI(TAG, "%d ms on / %d ms off, repeating. Watch for a start on EVERY cycle.",
             BENCH_ON_MS, BENCH_OFF_MS);
    ESP_LOGI(TAG, "Starting in 10 s.");

    motor_all_stop();
    vTaskDelay(pdMS_TO_TICKS(10000));

    for (uint32_t cycle = 1; ; cycle++) {
        ESP_LOGI(TAG, "cycle %lu: ON  (%.3f)", (unsigned long)cycle, (double)BENCH_THRUST);
        bench_drive(true);
        vTaskDelay(pdMS_TO_TICKS(BENCH_ON_MS));

        bench_drive(false);
        vTaskDelay(pdMS_TO_TICKS(BENCH_OFF_MS));
    }
}
#endif  // MOTOR_BENCH_MODE

// (motor driver, attitude estimator, flight control, UART) is FATAL and returns from app_main,
// while the navigation sensors are non-fatal and simply leave the outer control loops disengaged.
void app_main(void)
{
    ESP_LOGI(TAG, "System starting up...");

    esp_err_t error;

    imu_data_mutex = xSemaphoreCreateMutex();
    if (imu_data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create IMU data mutex");
        return;
    }

    imu_ready_semaphore = xSemaphoreCreateBinary();
    if (imu_ready_semaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create IMU ready semaphore");
        return;
    }

    // --- Motors first -------------------------------------------------------
    // Before anything else can command thrust, get the LEDC channels configured and driven to
    // zero. Note this does NOT protect against the gates floating between reset and this
    // line - that needs physical pull-downs, see the TODO in motor_driver.c.
    error = motor_driver_init();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Motor driver init failed: %s", esp_err_to_name(error));
        return;
    }

#if MOTOR_BENCH_MODE
    // Bench mode owns the board from here. Nothing below this line runs, so no sensor, no
    // estimator and no control loop can touch the motors while the ramp is measuring them.
    motor_bench_run();
    return;
#endif

    // --- Battery monitor -----------------------------------------------------
    // NON-FATAL by design, unlike everything else in this block. Pack voltage is telemetry, not
    // control - nothing in the cascade reads it - so a dead ADC or an unfitted divider must not
    // stop the aircraft flying. On failure battery_monitor_get_volts() reports the same 3.8 V
    // nominal that flight_control.c hardcoded before the divider existed, which is exactly the
    // behaviour this firmware had for its whole life up to 2026-09-03.
    error = battery_monitor_init();
    if (error != ESP_OK) {
        ESP_LOGW(TAG, "Battery monitor unavailable (%s) - reporting nominal voltage",
                 esp_err_to_name(error));
    }

    // --- Attitude estimator --------------------------------------------------
    // tau = 0.5 s: the gyro carries the short-term response, the accelerometer trims the
    // long-term drift out.
    error = attitude_init(1.0f / 1000.0f, 0.5f);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Attitude estimator init failed: %s", esp_err_to_name(error));
        return;
    }

    // --- Flight control ------------------------------------------------------
    error = flight_control_init();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Flight control init failed: %s", esp_err_to_name(error));
        return;
    }

    // --- Telemetry link ------------------------------------------------------
    telemetry_uart_config_t uart_config = {
        .uart_port  = UART_NUM_1,
        .tx_pin     = GPIO_NUM_6,
        .rx_pin     = GPIO_NUM_7,
        .baud_rate  = 460800
    };

    error = uart_telemetry_init(&uart_config);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize UART telemetry: %s", esp_err_to_name(error));
        return;
    }
    else {
        ESP_LOGI(TAG, "UART telemetry initialized successfully.");
    }

    // --- Control loop --------------------------------------------------------
    xTaskCreatePinnedToCore(
        fc_task,
        "Flight_Ctrl_Task",
        4096,
        NULL,
        configMAX_PRIORITIES - 1, //prioritatea cea mai mare, pentru a nu fi intrerupt de alte taskuri
        NULL,
        1
    );
    //Why core 1 for FC task ?
    //Core 0 -> PRO_CPU(PROTOCOL CPU): does run many background tasks like memory access, other OS related functions etc...
    //Core 1 -> APP_CPU(APPLICATION CPU): does run the application code, so we want our flight control task to run on this core to avoid any interruptions from background tasks.

    // --- Navigation sensors --------------------------------------------------
    // Wait for fc_task to finish imu_setup() and the calibration sweeps, because the VL53L1X
    // attaches to the I2C bus that imu_setup() creates. Accel calibration is 500 samples at
    // 2 ms (~1.5 s with the I2C reads), and the gyro sweep is 1000 samples and may retry up to
    // three times if it detects motion. The 2 ms delay rounds up at a 1 kHz tick and the I2C
    // read adds ~0.5 ms, so each sweep is nearer 3 s than 2 s: ~11 s worst case in total.
    // 30 s leaves room for that without the ToF ever starting mid-calibration.
    if (xSemaphoreTake(imu_ready_semaphore, pdMS_TO_TICKS(30000)) != pdTRUE) {
        ESP_LOGE(TAG, "IMU never became ready - navigation sensors will not be started");
    } else {
        error = sensor_task_init();
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "Sensor task init failed: %s", esp_err_to_name(error));
        } else {
            error = sensor_task_start(0);
            if (error != ESP_OK) {
                ESP_LOGE(TAG, "Failed to start sensor task: %s", esp_err_to_name(error));
            }
        }
    }

    xTaskCreatePinnedToCore(
        telemetry_task,
        "Telemetry_Task",
        4096,
        NULL,
        5,
        NULL,
        0
    );
}
