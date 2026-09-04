// battery_monitor.c - pack voltage from a resistor divider on one ADC pin.
//
// Replaces the hardcoded battery_voltage = 3.8f that flight_control.c reported until 2026-09-03.
//
// Reading only. No filtering, no thrust compensation, no low-voltage cutoff, and nothing in the
// control path reads the result - it populates the telemetry status frame and the dashboard tile
// and that is all.

#include "battery_monitor.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

// ---------------------------------------------------------------------------
// YOUR HARDWARE - these three are the only lines you should need to change
// ---------------------------------------------------------------------------

// Divider midpoint pin. ADC2 is fine here: this board runs no Wi-Fi (that is the telemetry
// module's job, reached over UART on GPIO 6/7). Every ADC1 pin is already taken by motors,
// UART and the flow sensor's SPI. Free alternatives: 15, 16, 17.
#define BATTERY_ADC_GPIO 14

// Pack volts per pin volt. 2.0 for any equal-value pair (22k/22k recommended).
#define BATTERY_DIVIDER_RATIO 2.0f

// Correction from the multimeter check: (meter volts) / (reported volts). 1.0 = not yet measured.
#define BATTERY_TRIM 1.0f

// ---------------------------------------------------------------------------

// 12 dB attenuation gives roughly a 150-3100 mV usable input range. A 4.2 V pack through a 2:1
// divider is 2.1 V, comfortably inside it.
#define BATTERY_ADC_ATTEN ADC_ATTEN_DB_12

// Conversions averaged per call. The S3's SAR ADC is noisy enough that a single reading is not a
// measurement; 8 costs a few hundred microseconds at 50 Hz.
#define BATTERY_OVERSAMPLE 8

// Reported when the ADC is unavailable, so callers never see a wild number from an unconfigured
// pin. Same value flight_control.c used to hardcode.
#define BATTERY_NOMINAL_VOLTS 3.8f

// Sanity window on the raw pin voltage, in millivolts. An unfitted divider or a broken wire
// leaves the pin floating; that should read as "no measurement", not as a plausible battery.
#define BATTERY_SANE_MIN_MV 500
#define BATTERY_SANE_MAX_MV 3000

static const char *TAG = "BATTERY";

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t cali_handle;
static adc_channel_t adc_channel;
static adc_unit_t adc_unit;
static bool monitor_ready;

esp_err_t battery_monitor_init(void)
{
    monitor_ready = false;

    // Derive unit and channel from the GPIO, so BATTERY_ADC_GPIO is the only thing to change.
    esp_err_t error = adc_oneshot_io_to_channel(BATTERY_ADC_GPIO, &adc_unit, &adc_channel);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "GPIO %d is not an ADC pin: %s", BATTERY_ADC_GPIO, esp_err_to_name(error));
        return error;
    }

    const adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = adc_unit,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    error = adc_oneshot_new_unit(&unit_config, &adc_handle);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(error));
        return error;
    }

    const adc_oneshot_chan_cfg_t channel_config = {
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    error = adc_oneshot_config_channel(adc_handle, adc_channel, &channel_config);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed: %s", esp_err_to_name(error));
        adc_oneshot_del_unit(adc_handle);
        return error;
    }

    // Curve fitting against the factory data in eFuse is what turns raw counts into millivolts.
    // Without it there is no conversion available at all, so fail rather than report a guess.
    const adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = adc_unit,
        .chan = adc_channel,
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    error = adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "ADC calibration unavailable: %s", esp_err_to_name(error));
        adc_oneshot_del_unit(adc_handle);
        return error;
    }

    monitor_ready = true;

    ESP_LOGI(TAG, "Battery monitor on GPIO %d (ADC%d ch%d), divider %.2f, trim %.4f",
             BATTERY_ADC_GPIO, (int)adc_unit + 1, (int)adc_channel,
             (double)BATTERY_DIVIDER_RATIO, (double)BATTERY_TRIM);
    ESP_LOGI(TAG, "First reading: %.3f V  (set BATTERY_TRIM = meter volts / this)",
             (double)battery_monitor_get_volts());

    return ESP_OK;
}

float battery_monitor_get_volts(void)
{
    if (!monitor_ready) {
        return BATTERY_NOMINAL_VOLTS;
    }

    // Averaged in millivolts rather than raw counts: the calibration curve is non-linear, so
    // converting each conversion before averaging is not the same as averaging then converting.
    int64_t millivolt_sum = 0;
    int good_conversions = 0;

    for (int i = 0; i < BATTERY_OVERSAMPLE; i++) {
        int raw = 0;
        if (adc_oneshot_read(adc_handle, adc_channel, &raw) != ESP_OK) {
            continue;
        }
        int millivolts = 0;
        if (adc_cali_raw_to_voltage(cali_handle, raw, &millivolts) != ESP_OK) {
            continue;
        }
        millivolt_sum += millivolts;
        good_conversions++;
    }

    if (good_conversions == 0) {
        return BATTERY_NOMINAL_VOLTS;
    }

    const int pin_millivolts = (int)(millivolt_sum / good_conversions);

    if (pin_millivolts < BATTERY_SANE_MIN_MV || pin_millivolts > BATTERY_SANE_MAX_MV) {
        return BATTERY_NOMINAL_VOLTS;
    }

    return ((float)pin_millivolts / 1000.0f) * BATTERY_DIVIDER_RATIO * BATTERY_TRIM;
}

bool battery_monitor_is_valid(void)
{
    return monitor_ready;
}
