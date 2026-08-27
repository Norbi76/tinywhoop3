// main.c - PMW3901 optical flow scale calibration bench tool.
//
// WHAT THIS PROGRAM DOES
//   Measures FLOW_COUNTS_PER_RAD for ../flight_controller/components/nav_estimator/nav_estimator.c,
//   which currently holds the guess 500.0f. That constant is the single scale factor turning
//   sensor counts into velocity, so everything about velocity hold depends on it.
//
//   It also gives you the TRANSLATION half of the axis/sign check that nav_estimator.c asks for:
//   sliding forward must produce positive delta_x, sliding right positive delta_y. The ROTATION
//   half (pitch in place -> velocity_x stays ~0) needs the IMU and belongs on the drone.
//
// THE MATH, AND A CORRECTION TO THE DOCUMENTED PROCEDURE
//   The recipe in nav_estimator.c says angle_moved_rad = atan2(D, h). That is wrong. The sensor
//   accumulates frame-to-frame image shifts, so a slide of distance D at height h accumulates
//   the integral of (v/h)dt = D/h - not atan(D/h), which is the angle subtended at the END
//   position and is not what gets accumulated.
//
//   It is also what the downstream code requires. nav_estimator computes
//       velocity = (counts / K) / dt * altitude
//   and for that to recover the true D/dt you need counts/K == D/h. So:
//
//       K = accumulated_counts * h / D
//
//   With the old recipe's own numbers (D=200, h=300) atan gives 0.588 rad against the correct
//   0.667 - K comes out 13% high and estimated velocity 13% low. Keeping D/h <= ~0.3 makes the
//   two forms agree to about 1%, so the rig below is specified that way and the distinction
//   cannot bite regardless of lens non-idealities.
//
// HOW A TRIAL WORKS
//   There is no countdown and no fixed window. Each trial ends itself once the rig has clearly
//   moved and then been stationary for CAL_QUIET_MS. Slide mark to mark in about a second, let
//   go, read the result, then slide it back - the return is simply the next trial, in the
//   opposite direction, and equally valid. Take the magnitude. Run several and average.
//
//   SLIDE BRISKLY. The sensor reports INTEGER counts, so a slide slow enough that each read sees
//   a fraction of a count can lose displacement and bias K low. The report prints counts-per-read
//   and warns below 0.5.
//
// WHY IT DOES NOT USE nav_estimator
//   nav_estimator gates flow on altitude_valid and would drag the ToF in. This talks to
//   pmw3901_driver directly and takes the height from a ruler instead.
//
// NO GYRO COMPENSATION HERE
//   The drone subtracts measured rotation from the flow. There is no IMU on this rig, so the
//   slide MUST be pure translation - keep the sensor flat and do not let the fixture rotate or
//   tilt. Any rotation goes straight into the measured constant as error.

#include <stdio.h>
#include <stdlib.h>   // abs()
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include "pmw3901_driver.h"

static const char *TAG = "FLOW_CAL";

// ---------------------------------------------------------------------------
// THE RIG. Both numbers go straight into the result (K = counts * h / D), so a 2% ruler error
// is a 2% error in the constant. Measure both carefully and keep them consistent in mm.
//
// CAL_HEIGHT_MM  - from the PMW3901's LENS to the floor, not from the PCB or the board edge.
// CAL_SLIDE_MM   - the distance between your two marks.
//
// Keep CAL_SLIDE_MM / CAL_HEIGHT_MM <= ~0.3 (see the header comment). At the defaults below
// that ratio is 100/400 = 0.25.
// ---------------------------------------------------------------------------
#define CAL_HEIGHT_MM 134.0f
#define CAL_SLIDE_MM  30.0f

// Poll period. Matches the drone's sensor_task rate so the per-read delta magnitudes are
// representative of what the flight code actually sees.
#define CAL_POLL_MS 10

// Surface-quality floor used only for REPORTING here, not for gating. This is the same guess
// (PMW3901_MIN_SQUAL) the driver applies to its `valid` flag.
#define CAL_SQUAL_ADVISORY 40

// ---------------------------------------------------------------------------
// FRAME VIEW MODE. Set to 1, rebuild and reflash to see what the sensor actually sees, as
// ASCII art, instead of running calibration trials.
//
// Use it to confirm nothing but the target surface is in frame. Anything at a different depth -
// a table edge, the vice or clamp holding the sensor - corrupts the correlation, and it does so
// while squal often still looks healthy, so you cannot detect it from the trial output.
//   - A nearer object flows FASTER than the target and biases K high.
//   - An object fixed to the sensor does not move at all, and biases K low.
//
// This is a SEPARATE BUILD on purpose: entering frame mode permanently disables motion
// reporting until the sensor is power-cycled, so the two modes cannot be mixed up at runtime.
// Set it back to 0 and reflash before running trials.
// ---------------------------------------------------------------------------
#define CAL_FRAME_VIEW 0

// Accumulated result of one slide.
typedef struct {
    int32_t sum_dx;         // int32, not int16 - a long slide overflows a 16-bit accumulator
    int32_t sum_dy;
    int32_t peak_dx;        // largest |sum_dx| REACHED during the slide, not the final value
    int32_t peak_dy;
    int      reads;         // reads while accumulating
    int      errors;        // failed reads
    int      low_squal;     // reads below the advisory threshold
    uint8_t  squal_min;
    uint32_t squal_sum;
    int32_t  peak_delta;    // largest |delta| in a SINGLE read - saturation watchdog
    int      duration_ms;   // how long the rig was actually moving
} trial_t;

// The sensor accumulates internally between reads and hands back int16 deltas. Slide fast enough
// and one read can saturate, which loses displacement and shows up as a quietly LOW K with no
// other symptom. Flag it rather than let it pass.
#define CAL_SATURATION_WARN 30000

// ---------------------------------------------------------------------------
// Automatic slide detection.
//
// The first version used a fixed 12 s window and asked you to slide, stop, and wait it out. That
// does not survive contact with reality: the return trip to the start mark lands inside the
// window, and since this sums NET displacement, the return cancels the slide. A run showing
// "+35 ... -51 ... +19" is one slide and one and a bit returns, not a measurement.
//
// So the window now ends itself. Accumulation runs continuously; once the rig has clearly moved
// and has then been stationary for CAL_QUIET_MS, the slide is over and the result prints. Move
// the rig back whenever you like - that just becomes the NEXT trial, in the opposite direction,
// which is equally valid. Take the magnitude.
// ---------------------------------------------------------------------------

// Cumulative |dx|+|dy| that counts as "the rig has definitely moved". Well clear of the
// jitter floor, and far below an expected slide of ~100.
#define CAL_MOTION_THRESHOLD 10

// ---- How "stopped" is detected, and why it is not the obvious test ----
// The first attempt called a read "stationary" if its delta was 0 or 1, and ended the slide
// after 1.2 s of those. That fails on real hardware: a stationary sensor does NOT report zeros.
// It jitters by +/-2..3 counts per read while the ACCUMULATED SUM stays put - one measured trace
// sat at dx 96..103 for twenty seconds without a single qualifying quiet period, and the slide
// ran into its 30 s cap.
//
// So the test is on the accumulated position instead: the slide is over once the sum has not
// moved more than CAL_SETTLE_TOLERANCE counts across a CAL_SETTLE_MS window. Jitter is zero-mean
// and cancels in the sum; real translation does not.
#define CAL_SETTLE_MS 1500
#define CAL_SETTLE_TOLERANCE 6
#define CAL_SETTLE_SAMPLES (CAL_SETTLE_MS / CAL_POLL_MS)

// Consequence worth knowing: a slide slow enough to move less than the tolerance per settle
// window is indistinguishable from jitter and will end early. At ~100 counts that means keeping
// the slide under about 20 s, which is not a demanding requirement - a second or two is ideal.

// Give up waiting for a slide to start.
#define CAL_ARM_TIMEOUT_MS 60000

// Hard cap on one slide, so a drifting rig cannot accumulate forever.
#define CAL_MAX_SLIDE_MS 30000

#if CAL_FRAME_VIEW

// Renders one captured frame as ASCII art.
//
// Two characters per pixel, because terminal cells are about twice as tall as they are wide and
// a 1:1 mapping would show a squashed image - which matters when you are judging whether a
// straight edge in shot is the table or the clamp.
//
// The ramp is auto-scaled between the frame's own min and max. The PMW3901's raw levels sit in
// a narrow band that shifts with lighting, so a fixed 0..63 ramp usually renders as flat grey.
static void render_frame(const uint8_t *pixels)
{
    static const char ramp[] = " .:-=+*#%@";
    const int ramp_levels = (int)(sizeof(ramp) - 2);   // exclude the NUL

    uint8_t low = 63, high = 0;
    for (int i = 0; i < PMW3901_FRAME_PIXELS; i++) {
        if (pixels[i] < low)  low  = pixels[i];
        if (pixels[i] > high) high = pixels[i];
    }

    const int span = (high > low) ? (high - low) : 1;

    // printf rather than ESP_LOGI: no per-line timestamp prefix, so the image stays aligned.
    printf("\n   +");
    for (int x = 0; x < PMW3901_FRAME_WIDTH * 2; x++) putchar('-');
    printf("+\n");

    for (int y = 0; y < PMW3901_FRAME_HEIGHT; y++) {
        printf("   |");
        for (int x = 0; x < PMW3901_FRAME_WIDTH; x++) {
            const int level = ((pixels[y * PMW3901_FRAME_WIDTH + x] - low) * ramp_levels) / span;
            const char c = ramp[level < 0 ? 0 : (level > ramp_levels ? ramp_levels : level)];
            putchar(c);
            putchar(c);
        }
        printf("|\n");
    }

    printf("   +");
    for (int x = 0; x < PMW3901_FRAME_WIDTH * 2; x++) putchar('-');
    printf("+\n");
    printf("   levels %u..%u  (dark = low reflectance)\n\n", low, high);
    fflush(stdout);
}

static void frame_view_loop(void)
{
    // 1225 bytes on the stack would be most of a default task stack; app_main has room, but
    // static keeps it obvious and costs nothing in a single-purpose tool.
    static uint8_t pixels[PMW3901_FRAME_PIXELS];

    ESP_LOGW(TAG, "");
    ESP_LOGW(TAG, "=== FRAME VIEW MODE =====================================");
    ESP_LOGW(TAG, "Motion reporting is DISABLED in this build. Set CAL_FRAME_VIEW");
    ESP_LOGW(TAG, "back to 0 and reflash to run calibration trials.");
    ESP_LOGW(TAG, "");
    ESP_LOGW(TAG, "What you want to see: uniform texture filling the whole frame.");
    ESP_LOGW(TAG, "What to look for and eliminate: a straight edge, a bright blob,");
    ESP_LOGW(TAG, "or a dark region that does not move when you slide the rig -");
    ESP_LOGW(TAG, "that is the table edge or the vice, and it will corrupt K.");
    ESP_LOGW(TAG, "=========================================================");

    esp_err_t error = pmw3901_frame_mode_enter();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Could not enter frame mode: %s", esp_err_to_name(error));
        return;
    }

    for (int frame = 1; ; frame++) {
        error = pmw3901_capture_frame(pixels, sizeof(pixels));
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "Frame capture failed: %s", esp_err_to_name(error));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "frame %d", frame);
        render_frame(pixels);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#else

static int32_t iabs32(int32_t v)
{
    return v < 0 ? -v : v;
}

// Accumulates one slide: waits for the rig to move, sums until it stops.
//
// Deliberately accumulates low-squal reads too. Dropping them would discard real displacement
// and bias the constant LOW; instead they are counted and reported so a bad surface shows up as
// a reason to re-run rather than as a silently wrong number.
//
// @return ESP_OK on a completed slide, ESP_ERR_TIMEOUT if nothing moved within the arm timeout.
static esp_err_t run_slide(trial_t *out)
{
    *out = (trial_t){ .squal_min = 255 };

    // Rolling history of the accumulated position, one entry per poll, so the settle test can
    // compare "where the sum is now" against "where it was CAL_SETTLE_MS ago".
    static int32_t history_dx[CAL_SETTLE_SAMPLES];
    static int32_t history_dy[CAL_SETTLE_SAMPLES];
    int history_index = 0;
    int history_filled = 0;

    bool moved = false;          // cleared the motion threshold at least once
    int  armed_ms = 0;           // time spent waiting for the slide to begin
    int  motion_start_ms = -1;   // when the rig first stirred, for the duration figure
    int  elapsed_ms = 0;
    bool settled = false;

    // vTaskDelayUntil, not vTaskDelay: a read blocks for ~1.4 ms of SPI, so a relative delay
    // would stretch the real period to ~11.4 ms. Cadence does not bias K - this sums
    // displacement, not rate - but it keeps the reported duration honest.
    TickType_t last_wake = xTaskGetTickCount();
    int last_print_ms = 0;

    for (;;) {
        pmw3901_motion_t motion;

        if (pmw3901_read_motion(&motion) == ESP_OK) {
            const int32_t activity = iabs32(motion.delta_x) + iabs32(motion.delta_y);

            out->sum_dx += motion.delta_x;
            out->sum_dy += motion.delta_y;
            out->reads++;
            out->squal_sum += motion.squal;

            if (motion.squal < out->squal_min) {
                out->squal_min = motion.squal;
            }
            if (motion.squal < CAL_SQUAL_ADVISORY) {
                out->low_squal++;
            }

            const int32_t biggest = iabs32(motion.delta_x) > iabs32(motion.delta_y)
                                        ? iabs32(motion.delta_x)
                                        : iabs32(motion.delta_y);
            if (biggest > out->peak_delta) {
                out->peak_delta = biggest;
            }

            // Peak EXCURSION, not final value. If the slide reverses, these diverge from the
            // net sums and report() says so - that is how a return trip is caught.
            if (iabs32(out->sum_dx) > out->peak_dx) out->peak_dx = iabs32(out->sum_dx);
            if (iabs32(out->sum_dy) > out->peak_dy) out->peak_dy = iabs32(out->sum_dy);

            if (motion_start_ms < 0 && activity > 0) {
                motion_start_ms = elapsed_ms;
            }
            if (!moved && (iabs32(out->sum_dx) + iabs32(out->sum_dy)) >= CAL_MOTION_THRESHOLD) {
                moved = true;
                ESP_LOGI(TAG, "  moving...");
            }

            // Has the accumulated position gone anywhere in the last CAL_SETTLE_MS?
            if (history_filled >= CAL_SETTLE_SAMPLES) {
                const int32_t drift = iabs32(out->sum_dx - history_dx[history_index]) +
                                      iabs32(out->sum_dy - history_dy[history_index]);
                settled = (drift <= CAL_SETTLE_TOLERANCE);
            }

            history_dx[history_index] = out->sum_dx;
            history_dy[history_index] = out->sum_dy;
            history_index = (history_index + 1) % CAL_SETTLE_SAMPLES;
            if (history_filled < CAL_SETTLE_SAMPLES) {
                history_filled++;
            }
        } else {
            out->errors++;
        }

        elapsed_ms += CAL_POLL_MS;

        if (moved) {
            if (settled) {
                break;                       // the sum has stopped moving - the slide is over
            }
            if (elapsed_ms >= CAL_MAX_SLIDE_MS) {
                ESP_LOGW(TAG, "  Slide hit the %d s cap without settling. If the trace above was",
                         CAL_MAX_SLIDE_MS / 1000);
                ESP_LOGW(TAG, "  flat, the number is still good - the rig just never went quiet.");
                break;
            }
        } else {
            armed_ms += CAL_POLL_MS;
            if (armed_ms >= CAL_ARM_TIMEOUT_MS) {
                return ESP_ERR_TIMEOUT;
            }
        }

        if (moved && (elapsed_ms - last_print_ms) >= 1000) {
            last_print_ms = elapsed_ms;
            ESP_LOGI(TAG, "    dx %+6ld  dy %+6ld", (long)out->sum_dx, (long)out->sum_dy);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CAL_POLL_MS));
    }

    // Exclude the trailing settle window so the figure reflects the movement itself.
    out->duration_ms = elapsed_ms - (settled ? CAL_SETTLE_MS : 0)
                                  - (motion_start_ms > 0 ? motion_start_ms : 0);
    if (out->duration_ms < CAL_POLL_MS) {
        out->duration_ms = CAL_POLL_MS;
    }

    return ESP_OK;
}

// K = |counts| * h / D. Sign is reported separately; the constant itself is a magnitude.
static float counts_to_k(int32_t counts)
{
    return (fabsf((float)counts) * CAL_HEIGHT_MM) / CAL_SLIDE_MM;
}

static void report(const trial_t *trial)
{
    const float k_from_x = counts_to_k(trial->sum_dx);
    const float k_from_y = counts_to_k(trial->sum_dy);
    const float squal_mean = trial->reads > 0 ? (float)trial->squal_sum / (float)trial->reads : 0.0f;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "--- trial result ----------------------------------------");
    ESP_LOGI(TAG, "  accumulated:  dx %+ld counts,  dy %+ld counts",
             (long)trial->sum_dx, (long)trial->sum_dy);
    ESP_LOGI(TAG, "  surface:      squal mean %.0f, min %u, %d/%d reads below %d",
             squal_mean, trial->squal_min, trial->low_squal, trial->reads, CAL_SQUAL_ADVISORY);

    if (trial->errors > 0) {
        ESP_LOGW(TAG, "  %d SPI reads failed - check wiring before trusting this trial.",
                 trial->errors);
    }
    if (trial->low_squal > trial->reads / 4) {
        ESP_LOGW(TAG, "  Over a quarter of reads were low-quality. The surface is too");
        ESP_LOGW(TAG, "  featureless - use something with visible texture and re-run.");
    }
    if (trial->peak_delta > CAL_SATURATION_WARN) {
        ESP_LOGW(TAG, "  Peak single-read delta %ld is near the int16 limit - you may have",
                 (long)trial->peak_delta);
        ESP_LOGW(TAG, "  saturated a read and LOST displacement, which biases K low.");
        ESP_LOGW(TAG, "  Slide more slowly and re-run.");
    }

    // A clean one-way slide ends at its own peak. If the net came back well below the peak, the
    // rig reversed mid-window - almost always the return trip starting too early - and the net
    // is the DIFFERENCE of two slides, not one slide.
    if (trial->peak_dx > 0 && iabs32(trial->sum_dx) < (trial->peak_dx * 4) / 5) {
        ESP_LOGW(TAG, "  REVERSED: dx peaked at %ld but ended at %+ld. The rig moved back before",
                 (long)trial->peak_dx, (long)trial->sum_dx);
        ESP_LOGW(TAG, "  it settled, so this net is not one slide. Discard this trial - let it");
        ESP_LOGW(TAG, "  come to a full stop and wait for the result before moving it back.");
    }

    // Duration is reported for context only. There was a warning here about slow slides losing
    // fractional counts to integer quantisation - MEASURED AND DISPROVED: across trials of 5.4 s
    // to 21.1 s over the same 30 mm, the accumulated counts were 105, 99, 104, 99 with no
    // correlation to duration. The sensor carries the fraction internally. Slide speed does not
    // bias the result; it only matters because a very slow slide is harder to tell from jitter
    // when deciding the slide has ended.
    ESP_LOGI(TAG, "  movement took %d ms", trial->duration_ms);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  If you slid along X (forward):  FLOW_COUNTS_PER_RAD = %.1f", k_from_x);
    ESP_LOGI(TAG, "  If you slid along Y (right):    FLOW_COUNTS_PER_RAD = %.1f", k_from_y);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  Sign check (nav_estimator expects forward -> +dx, right -> +dy):");
    ESP_LOGI(TAG, "     dx is %s, dy is %s",
             trial->sum_dx >= 0 ? "POSITIVE" : "NEGATIVE",
             trial->sum_dy >= 0 ? "POSITIVE" : "NEGATIVE");
    ESP_LOGI(TAG, "  Published figure for this part is near 500 - a sanity check, not a target.");
    ESP_LOGI(TAG, "---------------------------------------------------------");
}

#endif  // CAL_FRAME_VIEW

void app_main(void)
{
    ESP_LOGI(TAG, "PMW3901 flow scale calibration bench tool");
    ESP_LOGI(TAG, "SPI pins compiled in: SCLK=%d MISO=%d MOSI=%d CS=%d",
             PMW3901_PIN_SCLK, PMW3901_PIN_MISO, PMW3901_PIN_MOSI, PMW3901_PIN_CS);

#if !CAL_FRAME_VIEW
    ESP_LOGI(TAG, "Rig: height %.0f mm, slide %.0f mm (ratio %.2f)",
             CAL_HEIGHT_MM, CAL_SLIDE_MM, CAL_SLIDE_MM / CAL_HEIGHT_MM);
    ESP_LOGI(TAG, "Field of view at that height is a %.0f mm patch - the textured area",
             2.0f * CAL_HEIGHT_MM * 0.3839f);   // 2 * h * tan(21 deg)
    ESP_LOGI(TAG, "must cover all of it, with nothing else in shot.");

    if ((CAL_SLIDE_MM / CAL_HEIGHT_MM) > 0.35f) {
        // counts = K * D/h is exact for a pinhole over a plane, so a high ratio is not a
        // small-angle error as such. It matters because it makes the answer depend on whether
        // the sensor reports tangent units or true angle - unverified on this part - and
        // because it pushes the correlation out to where lens distortion is worst.
        ESP_LOGW(TAG, "Slide/height ratio is high. At this ratio the result becomes sensitive");
        ESP_LOGW(TAG, "to whether the sensor reports tangent or angle units, which is not");
        ESP_LOGW(TAG, "verified. Prefer a shorter slide or a greater height.");
    }
#endif

    esp_err_t error = pmw3901_init();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "pmw3901_init() failed: %s", esp_err_to_name(error));
        ESP_LOGE(TAG, "Check 3V3/GND and the four SPI pins above against your wiring.");
        return;
    }

    ESP_LOGI(TAG, "Sensor ready.");

#if CAL_FRAME_VIEW
    frame_view_loop();
    return;
#else
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Set the rig at %.0f mm over a TEXTURED surface (newspaper, patterned rug,",
             CAL_HEIGHT_MM);
    ESP_LOGI(TAG, "wood grain). Mark a start and an end %.0f mm apart.", CAL_SLIDE_MM);
    ESP_LOGI(TAG, "Keep the sensor FLAT - there is no gyro here, so any tilt or rotation");
    ESP_LOGI(TAG, "during the slide corrupts the measurement.");

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "No countdown - just slide. Each trial ends by itself once the rig has");
    ESP_LOGI(TAG, "moved and then been still for %.1f s. Slide mark to mark in about a", CAL_QUIET_MS / 1000.0f);
    ESP_LOGI(TAG, "second, let go, and wait for the result. Then slide it back: that is");
    ESP_LOGI(TAG, "simply the next trial, in the opposite direction. Take the magnitude.");

    for (int trial_number = 1; ; trial_number++) {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "=== TRIAL %d - waiting for the rig to move ==============", trial_number);

        trial_t trial;
        if (run_slide(&trial) != ESP_OK) {
            ESP_LOGW(TAG, "Nothing moved for %d s. Still waiting...", CAL_ARM_TIMEOUT_MS / 1000);
            trial_number--;
            continue;
        }

        report(&trial);
    }
#endif  // CAL_FRAME_VIEW
}
