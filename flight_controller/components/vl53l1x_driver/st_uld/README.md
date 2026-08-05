# ST VL53L1X Ultra Lite Driver

These are ST's unmodified ULD sources, vendored from **STSW-IMG009 v3.5.5** (`API/core/`).

```
st_uld/
├── VL53L1X_api.c / .h            ST, unmodified
├── VL53L1X_calibration.c / .h    ST, unmodified
└── LICENSE.txt                   ST, from API/LICENSE.txt
```

Licence: **BSD OPEN SOURCE SLA0103** (<https://www.st.com/SLA0103>) — permissive, fine to keep
in this repository. Do not edit these four files; if you need different behaviour, change the
wrapper in `../vl53l1x_driver.c` instead, so a future ULD update is a clean drop-in.

## What this project supplies around them

The ULD's entire porting contract is the nine functions declared in `../include/vl53l1_platform.h`
and implemented in `../vl53l1_platform.c`:

`VL53L1_WrByte` `VL53L1_WrWord` `VL53L1_WrDWord` `VL53L1_RdByte` `VL53L1_RdWord`
`VL53L1_RdDWord` `VL53L1_WriteMulti` `VL53L1_ReadMulti` `VL53L1_WaitMs`

They talk to the sensor over the same I2C bus `imu_driver` creates on `I2C_NUM_0`, using 16-bit
register indices sent MSB first. `imu_setup()` must therefore run before `vl53l1x_init()` — the
ordering is enforced by the `imu_ready_semaphore` handshake in `main.c`.

`VL53L1X_api.h` includes **only** `vl53l1_platform.h`, and defines `VL53L1X_ERROR` itself as
`uint8_t`. No `vl53l1_types.h` is required. (An earlier version of this component shipped a
minimal `vl53l1_types.h` shim written before these sources were available; it declared
`VL53L1X_ERROR` as `int8_t` and has been deleted, because it would have conflicted.)

## Build wiring

`../CMakeLists.txt` detects these files, adds them to `SRCS`, puts `st_uld/` first on the include
path, and defines `VL53L1X_ULD_PRESENT`. ST's sources are compiled with `-Wno-error` because they
are third-party and are not clean against ESP-IDF's warning settings.

If the files are ever removed, the component still builds: `vl53l1x_init()` returns
`ESP_ERR_NOT_SUPPORTED`, `nav_estimator` reports `altitude_valid == false`, `flight_control`
disengages the altitude and position loops, and the drone remains flyable in angle mode.

## Still to do

Run the offset calibration once and put the result in `VL53L1X_OFFSET_MM` in
`../vl53l1x_driver.c` — see the `TODO(bench)` comment there. `VL53L1X_CalibrateOffset()` from
`VL53L1X_calibration.c` is compiled in and ready to call: place a 17% grey target at exactly
140 mm and pass that distance in.
