// battery_monitor.h - pack voltage from a resistor divider on one ADC pin.
//
// Replaces the hardcoded battery_voltage = 3.8f that flight_control.c reported until 2026-09-03.
//
// READING ONLY. No thrust compensation, no low-voltage cutoff, and nothing in the control path
// reads the result - it fills the telemetry status frame and the dashboard tile, nothing else.
// Compensation and cutoffs are separate changes with their own verification; a threshold applied
// to an uncalibrated reading is worse than no threshold.
//
// Called at 50 Hz from flight_control_get_status(), which runs on the telemetry task.
// DO NOT call it from the 1 kHz control loop - the oversampled read is a few hundred
// microseconds and has no business inside a tick that must finish in 1 ms.

#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Configures the ADC unit, channel and calibration. Call once at boot.
//
// The GPIO, divider ratio and trim factor are compile-time constants at the top of
// battery_monitor.c - change them there.
//
// A failure here is NOT fatal: battery_monitor_get_volts() then reports the nominal fallback and
// the aircraft flies exactly as it did before this component existed.
esp_err_t battery_monitor_init(void);

// Pack voltage in volts, oversampled and calibrated. Unfiltered - it will jitter by tens of mV,
// and sag under motor load, because that is what the pack actually does.
//
// Returns 3.8 V if the ADC is unavailable, or if the pin reads outside a plausible window
// (an unfitted divider or a broken wire leaves it floating). A reading pinned at exactly 3.80
// therefore means "no measurement", not "3.8 volt battery".
float battery_monitor_get_volts(void);

// True once the ADC and its calibration came up. Note this says the hardware is working, NOT
// that BATTERY_TRIM has been measured - those are different conditions, and the dashboard tile
// should stay dimmed until both hold.
bool battery_monitor_is_valid(void);

#ifdef __cplusplus
}
#endif
