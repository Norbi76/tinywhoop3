#pragma once

// vl53l1_platform.h - declarations of the ULD porting contract. Implemented in vl53l1_platform.c.
//
// These signatures are dictated by ST, not by us: VL53L1X_api.c calls them by exactly these names
// and prototypes. Do not "improve" them (drop the unused `dev`, change the return type to
// esp_err_t, etc.) - ST's vendored .c files in st_uld/ will stop compiling.
//
// ESP32 platform layer for ST's VL53L1X Ultra Lite Driver (ULD).
//
// These nine functions are the entire porting contract of the ULD: ST's VL53L1X_api.c and
// VL53L1X_calibration.c call nothing else to reach the hardware. The implementations live in
// vl53l1_platform.c and go out over the same I2C bus the MPU6500 is already on.
//
// The `dev` argument carried through every call is the sensor's 7-bit I2C address. The ULD
// passes it around opaquely so a single build can drive several sensors on one bus; we only
// have one, but the signatures must match ST's exactly or their .c files will not compile.
//
// NOTE: register indices are 16-BIT for this part (unlike the MPU6500's 8-bit registers) and
// go out most-significant byte first.

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Writes `count` bytes starting at 16-bit register `index`.
// @return 0 on success, -1 on an I2C failure.
int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);

// Reads `count` bytes starting at 16-bit register `index`.
// @return 0 on success, -1 on an I2C failure.
int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);

// Writes a single byte to 16-bit register `index`.
// @return 0 on success, -1 on an I2C failure.
int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data);

// Writes a 16-bit word (big endian on the wire) to 16-bit register `index`.
// @return 0 on success, -1 on an I2C failure.
int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data);

// Writes a 32-bit dword (big endian on the wire) to 16-bit register `index`.
// @return 0 on success, -1 on an I2C failure.
int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data);

// Reads a single byte from 16-bit register `index`.
// @return 0 on success, -1 on an I2C failure.
int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data);

// Reads a 16-bit word (big endian on the wire) from 16-bit register `index`.
// @return 0 on success, -1 on an I2C failure.
int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data);

// Reads a 32-bit dword (big endian on the wire) from 16-bit register `index`.
// @return 0 on success, -1 on an I2C failure.
int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data);

// Blocks for `wait_ms` milliseconds.
// @return 0 always.
int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms);

#ifdef __cplusplus
}
#endif
