# battery_monitor

Pack voltage from a resistor divider on one ADC pin. Replaces the `battery_voltage = 3.8f`
constant that `flight_control.c` reported until 2026-09-03.

## Hardware

| | |
|---|---|
| Pin | **GPIO 14** (ADC2 channel 3) |
| Divider | 22 kΩ / 22 kΩ from pack+ to the pin to ground, ratio 2.0 |
| Decoupling | **100 nF** from the pin to ground |
| Attenuation | 12 dB, usable input roughly 150–3100 mV |

A 1S pack at 4.2 V through a 2:1 divider puts 2.1 V on the pin, with headroom at both ends.

### Why GPIO 14

ADC2 is unusable while Wi-Fi is running — which rules it out on the *telemetry module*, but this
board runs no Wi-Fi at all; it reaches the telemetry module over UART on GPIO 6/7. Every ADC1 pin
is already taken: motors on 1, 2, 4, 5; UART on 6, 7; PMW3901 SPI on 8, 9, 10. GPIO 3 is the only
free one and it is a strapping pin a divider would hold at half pack voltage through every boot.

Free alternatives: **15, 16, 17**. Avoid 19 and 20 (USB D-/D+). Changing `BATTERY_ADC_GPIO` is
sufficient — unit and channel are derived via `adc_oneshot_io_to_channel()`.

### Why 22k and not 100k

The 100 nF capacitor is Espressif's only *written* ADC recommendation — their ESP32-S3 schematic
checklist says to fit one, and the datasheet publishes no source-impedance limit at all. The
resistor choice is therefore ours to justify: 22 k/22 k gives an 11 kΩ Thévenin source and a
1.1 ms RC settling time, and draws 95 µA. 100 k/100 k would be 50 kΩ and 5 ms — probably still
fine given the cap, but with no specification to point at there is no reason to sit near the edge.

## Calibrating it

`BATTERY_TRIM` ships at `1.0f`, meaning *not yet measured*.

1. Power up on the pack, open the dashboard, read the **Batt** tile. (No serial needed — the board
   cannot take USB and pack power together.)
2. Multimeter across the pack terminals, motors off.
3. `BATTERY_TRIM = (meter volts) / (tile volts)`. Set it, reflash, confirm they agree.
4. Repeat near the bottom of a pack. A constant ratio means the trim is right; a ratio that
   changes means the divider is being loaded — fit smaller resistors.

The code has a scale factor and no offset, because a divider genuinely passes through zero. If one
ratio does not fit both ends, that is a hardware problem, not a second parameter to fit.

## How the number is made

Eight conversions, each converted to millivolts through the curve-fitting calibration, then
averaged. Per-conversion conversion matters because the calibration curve is non-linear —
averaging raw counts and converting once gives a different, wrong answer.

**No filtering.** The value jitters by tens of millivolts and sags under motor load, because that
is what the pack does. If a smoother display is wanted later, filter it in the dashboard rather
than here, so the log keeps the real signal.

If the calibration scheme is unavailable, `battery_monitor_init()` **fails** rather than
converting counts by a guess.

## The sanity gate

Pin readings outside 500–3000 mV return the 3.8 V nominal instead. A floating pin should read as
*no measurement*, not as a plausible battery.

Useful consequence: **a Batt tile pinned at exactly 3.80 V that never moves means the divider is
not working.** Anything else means it is reading.

## What is deliberately absent

No thrust compensation, no low-voltage cutoff, no warnings, and nothing in the control path reads
this. `motor_thrust_to_duty()` remains a plain linear map with no voltage term.

`battery_monitor_is_valid()` reports that the *hardware* works, not that the trim has been
measured. The dashboard tile is labelled "Batt (nom)" and dimmed; un-dim it once both hold.
