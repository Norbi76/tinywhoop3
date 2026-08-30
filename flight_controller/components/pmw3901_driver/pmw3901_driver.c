// pmw3901_driver.c - SPI transport and (partial) bring-up for the optical flow sensor.
//
// STATUS: the init sequence is now IN (pmw3901_init_registers.h, transcribed from Bitcraze), so
// this driver will initialise real hardware. It has NOT been run against a sensor yet, and the
// constant that turns its output into velocity - FLOW_COUNTS_PER_RAD in nav_estimator.c - is
// still a guess, as are the gyro-compensation axis signs.
//
// >>> CONSEQUENCE FOR FLIGHT: while init failed, velocity_valid was permanently false and the
// velocity outer loop could never engage. That was a safety property this project relied on.
// It is now gone. With a working ToF supplying altitude_valid, flight_control WILL engage
// velocity hold on an unmeasured scale factor and unverified signs - and a sign error there is
// positive feedback, i.e. the drone accelerates away instead of holding. Measure
// FLOW_COUNTS_PER_RAD and verify the signs on the bench (see ../../../flow_calibration/) before
// flying this. <<<
//
// WHAT THIS FILE DOES
//   1. pmw3901_cs_low/high()   manual chip select (see the block comment above them).
//   2. pmw3901_write_reg()     address byte with MSB SET, then the value, then wait tSWW.
//   3. pmw3901_read_reg()      address byte with MSB CLEAR, wait tSRAD, clock the data out,
//                              CS low across both halves, then wait tSRR.
//   4. pmw3901_init()          GPIO + SPI bring-up, power-up reset, two-ID identity check, stub.
//   5. pmw3901_read_motion()   six register reads -> deltas, squal, motion flag, validity gate.
//
// HOW IT DOES IT / WHY IT LOOKS LIKE THIS
//   The inter-transaction waits are esp_rom_delay_us() busy-waits, not vTaskDelay(): at tens of
//   microseconds a scheduler round trip costs more than the delay itself. That is one of the
//   reasons this component belongs on sensor_task (100 Hz, core 0) and must never be called from
//   the 1 kHz control loop.
//
//   The identity check requires BOTH product_id == 0x49 and inverse_product_id == 0xB6. A floating
//   MISO reads as a constant 0x00 or 0xFF, which could accidentally satisfy a single-ID check.
//
// THE TIMING CONSTANTS ARE UNVERIFIED - and violating PixArt's minimum gaps corrupts data
// SILENTLY rather than raising an error. See TODO(datasheet) below.

#include "pmw3901_driver.h"
#include "pmw3901_init_registers.h"   // vendored PixArt power-up table, see that file's licence
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "PMW3901";

// ---------------------------------------------------------------------------
// PINS CONFIRMED AGAINST THE AIRFRAME WIRING (2026-08-29). Verified by grep against the
// drivers themselves, not against a copied list - that list had drifted stale once before.
//
//   GPIO 1, 2, 4, 5   motors (RL, RR, FL, FR)      motor_driver.c:34-37
//   GPIO 6, 7         telemetry UART TX/RX          main.c:389-390
//   GPIO 11, 12       I2C SCL/SDA (IMU + ToF)       imu_driver.c:34-35
//   GPIO 13           VL53L1X XSHUT                 vl53l1x_driver.c:52
//   GPIO 8, 9, 10, 18 optical flow SPI              here
//
// >>> GPIO 18 IS NOT ON THE SUPER MINI'S SIDE HEADER. <<<
// The board only breaks out GPIO 1-13 on the side castellations, which is why every other
// peripheral above lives in that range. SCLK is soldered to an UNDERSIDE PAD. That joint has
// no strain relief and is the mechanically weakest connection on the aircraft.
//
// A lifted MISO pad reads as a floating line - constant 0x00 or 0xFF - which is exactly what
// the dual product-ID check in pmw3901_init() exists to catch. If the sensor stops identifying
// after a crash, suspect the solder joint before the code.
//
// Avoid, if these ever move: strapping pins (0, 3, 45, 46), USB (19, 20), SPI flash (26-32),
// UART0 console (43, 44), onboard RGB LED (48).
// ---------------------------------------------------------------------------
// Each is #ifndef-guarded so a BENCH HARNESS can override it from its own CMakeLists with
// target_compile_definitions, without editing (and having to un-edit) this file. The drone build
// defines none of them and therefore compiles bit-identically to before the guards were added.
// See ../../../flow_calibration/, which overrides all five to match whatever GPIOs the spare
// board exposes. Unlike the ToF, this driver CREATES its SPI bus rather than borrowing one, so
// the pins cannot be chosen by the caller at runtime.
#ifndef PMW3901_SPI_HOST
#define PMW3901_SPI_HOST SPI2_HOST
#endif
#ifndef PMW3901_PIN_SCLK
#define PMW3901_PIN_SCLK 18
#endif
#ifndef PMW3901_PIN_MISO
#define PMW3901_PIN_MISO 8
#endif
#ifndef PMW3901_PIN_MOSI
#define PMW3901_PIN_MOSI 9
#endif
#ifndef PMW3901_PIN_CS
#define PMW3901_PIN_CS   10
#endif

// SPI mode 3 (CPOL=1, CPHA=1) and 2 MHz maximum - both are PixArt datasheet requirements.
#define PMW3901_SPI_MODE 3
#define PMW3901_SPI_CLOCK_HZ 2000000

// Register map - only the handful this driver touches directly.
// The init sequence uses many more; those live in the stub below.
#define PMW3901_REG_PRODUCT_ID          0x00
#define PMW3901_REG_MOTION              0x02
#define PMW3901_REG_DELTA_X_L           0x03
#define PMW3901_REG_DELTA_X_H           0x04
#define PMW3901_REG_DELTA_Y_L           0x05
#define PMW3901_REG_DELTA_Y_H           0x06
#define PMW3901_REG_SQUAL               0x07
#define PMW3901_REG_POWER_UP_RESET      0x3A
#define PMW3901_REG_INVERSE_PRODUCT_ID  0x5F

#define PMW3901_PRODUCT_ID_EXPECTED         0x49
#define PMW3901_INVERSE_PRODUCT_ID_EXPECTED 0xB6
#define PMW3901_POWER_UP_RESET_VALUE        0x5A

// ---------------------------------------------------------------------------
// SPI inter-transaction delays. Violating PixArt's minimum gaps produces SILENT DATA CORRUPTION
// rather than an error, which is miserable to debug and dangerous to fly on.
//
// These now match Bitcraze's registerWrite()/registerRead() exactly - the same source the init
// table came from. They previously did NOT, despite a comment claiming they did: the old values
// were 1 us inside CS on both sides and 45 us after, against Bitcraze's 50/50/200. Partially
// adopting a known-working bring-up is exactly how you get a sensor that returns confident
// garbage, so the delays and the register table are kept in lockstep.
//
// STILL TODO(datasheet): these are matched to a working implementation, NOT verified against a
// PixArt datasheet - there is no PMW3901 datasheet in documents/. Bitcraze's numbers are more
// conservative than the values usually quoted for this part family, so the risk is wasted
// microseconds, not corruption.
//
// Cost: a register read is ~208 us, and pmw3901_read_motion() does six of them, so ~1.25 ms per
// call against sensor_task's 10 ms budget. That is why the burst-read register is still not
// worth the extra failure mode.
// ---------------------------------------------------------------------------
#define PMW3901_TIMING_CS_SETTLE_US  50   // after CS falls, before the first SCLK edge
#define PMW3901_TIMING_WRITE_HOLD_US 50   // after a write's last SCLK edge, before CS rises
#define PMW3901_TIMING_WRITE_GAP_US  200  // after CS rises following a write
#define PMW3901_TIMING_TSRAD_US      50   // read: address byte -> data byte
#define PMW3901_TIMING_READ_HOLD_US  100  // after a read's data byte, before CS rises

// ---------------------------------------------------------------------------
// TODO(bench): Minimum usable surface quality.
// Below this, the floor is too featureless (plain carpet, glossy laminate, a white sheet of
// paper) for the flow reading to mean anything, and nav_estimator gates velocity out.
// Find the real threshold by flying low over the actual surfaces you plan to shoot over and
// logging squal. 40 is a common starting point but it is a guess.
// ---------------------------------------------------------------------------
#define PMW3901_MIN_SQUAL 40

static bool sensor_available;
static spi_device_handle_t spi_handle;

// --- Manual chip select -----------------------------------------------------
// Hardware CS is deliberately NOT used. The PMW3901 needs CS held low across a
// write-address-then-delay-then-read-data sequence; the ESP32's hardware CS deasserts at the
// end of each transaction, which would break that sequence. So spics_io_num is -1 and we
// drive the line ourselves.
// The settle delay lives here (rather than in the callers) because every transaction needs it.
// The trailing delays differ between reads and writes, so those stay at the call sites.
static void pmw3901_cs_low(void) {
    gpio_set_level(PMW3901_PIN_CS, 0);
    esp_rom_delay_us(PMW3901_TIMING_CS_SETTLE_US);
}

static void pmw3901_cs_high(void) {
    gpio_set_level(PMW3901_PIN_CS, 1);
}

// Writes one register. The MSB of the address byte set to 1 marks a write.
static esp_err_t pmw3901_write_reg(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), value };

    spi_transaction_t transaction = {
        .length = 8 * sizeof(tx),
        .tx_buffer = tx,
    };

    pmw3901_cs_low();
    esp_err_t error = spi_device_polling_transmit(spi_handle, &transaction);
    esp_rom_delay_us(PMW3901_TIMING_WRITE_HOLD_US);
    pmw3901_cs_high();

    esp_rom_delay_us(PMW3901_TIMING_WRITE_GAP_US);

    if (error != ESP_OK) {
        ESP_LOGE(TAG, "SPI write to reg 0x%02x failed: %s", reg, esp_err_to_name(error));
    }
    return error;
}

// Reads one register. The MSB of the address byte cleared to 0 marks a read.
static esp_err_t pmw3901_read_reg(uint8_t reg, uint8_t *value) {
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t address = (uint8_t)(reg & 0x7F);

    spi_transaction_t address_transaction = {
        .length = 8,
        .tx_buffer = &address,
    };

    uint8_t rx = 0;
    spi_transaction_t data_transaction = {
        .length = 8,
        .rxlength = 8,
        .rx_buffer = &rx,
    };

    // CS stays low across both halves - this is the reason for manual CS.
    pmw3901_cs_low();

    esp_err_t error = spi_device_polling_transmit(spi_handle, &address_transaction);
    if (error == ESP_OK) {
        esp_rom_delay_us(PMW3901_TIMING_TSRAD_US);
        error = spi_device_polling_transmit(spi_handle, &data_transaction);
        esp_rom_delay_us(PMW3901_TIMING_READ_HOLD_US);
    }

    // No gap after CS rises: Bitcraze have that delay commented out, and the next transaction
    // opens with its own CS_SETTLE anyway. Matching them exactly is the point.
    pmw3901_cs_high();

    if (error != ESP_OK) {
        ESP_LOGE(TAG, "SPI read of reg 0x%02x failed: %s", reg, esp_err_to_name(error));
        return error;
    }

    *value = rx;
    return ESP_OK;
}

// Writes one table of register/value pairs, stopping at the first SPI failure.
static esp_err_t pmw3901_write_table(const pmw3901_reg_write_t *table, size_t count) {
    for (size_t i = 0; i < count; i++) {
        esp_err_t error = pmw3901_write_reg(table[i].reg, table[i].value);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "Init sequence failed at entry %u (reg 0x%02x)",
                     (unsigned)i, table[i].reg);
            return error;
        }
    }
    return ESP_OK;
}

// PixArt's power-up configuration: 73 register writes in two halves around a mandatory 100 ms
// pause. The values live in pmw3901_init_registers.h - see that file for provenance and licence.
// They are undocumented magic from a PixArt application note; do not edit them here.
static esp_err_t pmw3901_write_init_sequence(void) {
    esp_err_t error = pmw3901_write_table(pmw3901_init_part1,
                                          sizeof(pmw3901_init_part1) / sizeof(pmw3901_init_part1[0]));
    if (error != ESP_OK) {
        return error;
    }

    vTaskDelay(pdMS_TO_TICKS(PMW3901_INIT_PAUSE_MS));

    return pmw3901_write_table(pmw3901_init_part2,
                               sizeof(pmw3901_init_part2) / sizeof(pmw3901_init_part2[0]));
}

esp_err_t pmw3901_init(void) {
    if (sensor_available) {
        return ESP_OK;
    }

    esp_err_t error;

    // Manual CS line, idle high.
    const gpio_config_t cs_config = {
        .pin_bit_mask = (1ULL << PMW3901_PIN_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    error = gpio_config(&cs_config);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "CS gpio_config failed: %s", esp_err_to_name(error));
        return error;
    }
    gpio_set_level(PMW3901_PIN_CS, 1);

    const spi_bus_config_t bus_config = {
        .sclk_io_num = PMW3901_PIN_SCLK,
        .mosi_io_num = PMW3901_PIN_MOSI,
        .miso_io_num = PMW3901_PIN_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };

    error = spi_bus_initialize(PMW3901_SPI_HOST, &bus_config, SPI_DMA_DISABLED);
    if (error != ESP_OK && error != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE just means the bus is already up, which is fine.
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(error));
        return error;
    }

    const spi_device_interface_config_t device_config = {
        .mode = PMW3901_SPI_MODE,
        .clock_speed_hz = PMW3901_SPI_CLOCK_HZ,
        .spics_io_num = -1,        // manual CS, see pmw3901_cs_low()
        .queue_size = 1,
        .command_bits = 0,
        .address_bits = 0,
    };

    error = spi_bus_add_device(PMW3901_SPI_HOST, &device_config, &spi_handle);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(error));
        return error;
    }

    // Reset the sensor's SPI port state machine before touching a single register. The part can
    // power up with its SPI interface mid-transaction (a partially clocked byte from supply
    // ramp), and toggling CS is the documented way to resynchronise it. Bitcraze do this first
    // thing in begin(); it was missing here.
    gpio_set_level(PMW3901_PIN_CS, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(PMW3901_PIN_CS, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(PMW3901_PIN_CS, 1);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Power-up reset, then let the part settle.
    error = pmw3901_write_reg(PMW3901_REG_POWER_UP_RESET, PMW3901_POWER_UP_RESET_VALUE);
    if (error != ESP_OK) {
        return error;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // Identity check. Both IDs must match or we are talking to the wrong thing (or nothing).
    uint8_t product_id = 0, inverse_product_id = 0;
    error = pmw3901_read_reg(PMW3901_REG_PRODUCT_ID, &product_id);
    if (error != ESP_OK) {
        return error;
    }
    error = pmw3901_read_reg(PMW3901_REG_INVERSE_PRODUCT_ID, &inverse_product_id);
    if (error != ESP_OK) {
        return error;
    }

    if (product_id != PMW3901_PRODUCT_ID_EXPECTED ||
        inverse_product_id != PMW3901_INVERSE_PRODUCT_ID_EXPECTED) {
        ESP_LOGE(TAG, "PMW3901 not found: product_id=0x%02x (expected 0x%02x), inverse=0x%02x (expected 0x%02x)",
                 product_id, PMW3901_PRODUCT_ID_EXPECTED,
                 inverse_product_id, PMW3901_INVERSE_PRODUCT_ID_EXPECTED);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "PMW3901 detected (product_id=0x%02x)", product_id);

    // Read the motion registers once and discard the result. This clears the motion latch and
    // whatever deltas accumulated during power-up, so the first real read reports movement since
    // init rather than since the supply came up. Bitcraze do this before initRegisters(); it was
    // missing here. Errors are ignored on purpose - the values are being thrown away.
    uint8_t discard = 0;
    (void)pmw3901_read_reg(PMW3901_REG_MOTION, &discard);
    (void)pmw3901_read_reg(PMW3901_REG_DELTA_X_L, &discard);
    (void)pmw3901_read_reg(PMW3901_REG_DELTA_X_H, &discard);
    (void)pmw3901_read_reg(PMW3901_REG_DELTA_Y_L, &discard);
    (void)pmw3901_read_reg(PMW3901_REG_DELTA_Y_H, &discard);
    vTaskDelay(pdMS_TO_TICKS(1));

    error = pmw3901_write_init_sequence();
    if (error != ESP_OK) {
        ESP_LOGW(TAG, "Optical flow disabled; the drone will fly without velocity hold.");
        return error;
    }

    sensor_available = true;
    ESP_LOGI(TAG, "PMW3901 ready");

    return ESP_OK;
}

esp_err_t pmw3901_read_motion(pmw3901_motion_t *out) {
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sensor_available) {
        return ESP_ERR_INVALID_STATE;
    }

    // Individual register reads rather than the burst-read register. Slower, but the burst
    // buffer layout is one more thing to get wrong, and at 100 Hz there is time to spare.
    uint8_t motion = 0, dx_l = 0, dx_h = 0, dy_l = 0, dy_h = 0, squal = 0;

    esp_err_t error = pmw3901_read_reg(PMW3901_REG_MOTION, &motion);
    if (error == ESP_OK) error = pmw3901_read_reg(PMW3901_REG_DELTA_X_L, &dx_l);
    if (error == ESP_OK) error = pmw3901_read_reg(PMW3901_REG_DELTA_X_H, &dx_h);
    if (error == ESP_OK) error = pmw3901_read_reg(PMW3901_REG_DELTA_Y_L, &dy_l);
    if (error == ESP_OK) error = pmw3901_read_reg(PMW3901_REG_DELTA_Y_H, &dy_h);
    if (error == ESP_OK) error = pmw3901_read_reg(PMW3901_REG_SQUAL, &squal);

    if (error != ESP_OK) {
        out->valid = false;
        return error;
    }

    out->delta_x = (int16_t)(((uint16_t)dx_h << 8) | dx_l);
    out->delta_y = (int16_t)(((uint16_t)dy_h << 8) | dy_l);
    out->squal = squal;
    out->motion = (motion & 0x80) != 0;   // bit 7 = MOT, motion since last read
    out->valid = (squal >= PMW3901_MIN_SQUAL);

    return ESP_OK;
}

bool pmw3901_is_available(void) {
    return sensor_available;
}

// ===========================================================================
// RAW FRAME READOUT - BENCH DIAGNOSTIC ONLY. NOTHING IN THE FLIGHT PATH CALLS THIS.
//
// The PMW3901 can stream its raw 35x35 image out over SPI. That is useful for exactly one
// thing: checking what is actually inside the sensor's field of view before trusting a
// measurement, since anything at a different depth from the target plane (a table edge, the
// clamp holding the sensor) corrupts the correlation while often leaving squal looking healthy.
//
// >>> ENTERING FRAME MODE STOPS MOTION REPORTING. <<<
// The register writes below reconfigure the part for image readout. pmw3901_read_motion() is
// meaningless afterwards, and there is no software path back - power-cycle the sensor and let
// pmw3901_init() run again. That is why this is a separate mode rather than something you can
// interleave with ranging, and why ../../../flow_calibration/ selects it at COMPILE time.
//
// The register sequence and the two-byte pixel protocol are Bitcraze's
// enableFrameBuffer()/readFrameBuffer() - see pmw3901_init_registers.h for provenance and
// licence. The bounded retry counts are ours: Bitcraze spin forever, which is not acceptable
// even in a bench tool.
// ===========================================================================

// Status register the frame data streams out of, and its ready encoding in bits 7:6.
#define PMW3901_REG_FRAME_STATUS 0x58
#define PMW3901_FRAME_STATUS_UPPER 0x01   // this byte carries the upper 6 bits of a pixel

// Bounded spin limits. A healthy sensor satisfies these in a handful of reads.
#define PMW3901_FRAME_READY_RETRIES 1000
#define PMW3901_FRAME_PIXEL_RETRIES 100

// Polls the status register until it leaves the "busy" encoding.
static esp_err_t pmw3901_frame_wait_ready(void) {
    for (int attempt = 0; attempt < PMW3901_FRAME_READY_RETRIES; attempt++) {
        uint8_t status = 0;
        esp_err_t error = pmw3901_read_reg(PMW3901_REG_FRAME_STATUS, &status);
        if (error != ESP_OK) {
            return error;
        }
        if ((status >> 6) != 0x03) {
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG, "Frame buffer never signalled ready");
    return ESP_ERR_TIMEOUT;
}

esp_err_t pmw3901_frame_mode_enter(void) {
    if (!sensor_available) {
        return ESP_ERR_INVALID_STATE;
    }

    static const pmw3901_reg_write_t frame_mode[] = {
        { 0x7F, 0x07 }, { 0x41, 0x1D }, { 0x4C, 0x00 }, { 0x7F, 0x08 },
        { 0x6A, 0x38 }, { 0x7F, 0x00 }, { 0x55, 0x04 }, { 0x40, 0x80 },
        { 0x4D, 0x11 }, { 0x70, 0x00 }, { 0x58, 0xFF },
    };

    esp_err_t error = pmw3901_write_table(frame_mode, sizeof(frame_mode) / sizeof(frame_mode[0]));
    if (error != ESP_OK) {
        return error;
    }

    error = pmw3901_frame_wait_ready();
    if (error != ESP_OK) {
        return error;
    }

    esp_rom_delay_us(50);
    ESP_LOGW(TAG, "Frame readout mode active - motion reporting is now DISABLED.");
    return ESP_OK;
}

esp_err_t pmw3901_capture_frame(uint8_t *pixels, size_t pixel_count) {
    if (pixels == NULL || pixel_count < PMW3901_FRAME_PIXELS) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sensor_available) {
        return ESP_ERR_INVALID_STATE;
    }

    // Pixels arrive as a pair of status-tagged bytes. Bits 7:6 say what the byte is:
    // 0x03/0x00 = not ready, re-read; 0x01 = this byte holds a pixel's upper 6 bits, and the
    // NEXT read holds its low bits. We keep only the upper 6 (0..63) - plenty for a visual
    // check, and it sidesteps an ambiguity in how Bitcraze pack the low 2 bits back in. The
    // second byte must still be consumed or the stream desynchronises.
    for (size_t i = 0; i < PMW3901_FRAME_PIXELS; i++) {
        uint8_t upper = 0;
        bool got = false;

        for (int attempt = 0; attempt < PMW3901_FRAME_PIXEL_RETRIES && !got; attempt++) {
            esp_err_t error = pmw3901_read_reg(PMW3901_REG_FRAME_STATUS, &upper);
            if (error != ESP_OK) {
                return error;
            }
            got = ((upper >> 6) == PMW3901_FRAME_STATUS_UPPER);
        }

        if (!got) {
            ESP_LOGE(TAG, "Frame stream stalled at pixel %u", (unsigned)i);
            return ESP_ERR_TIMEOUT;
        }

        uint8_t lower = 0;
        esp_err_t error = pmw3901_read_reg(PMW3901_REG_FRAME_STATUS, &lower);
        if (error != ESP_OK) {
            return error;
        }

        pixels[i] = (uint8_t)(upper & 0x3F);
    }

    // Re-arm for the next frame.
    esp_err_t error = pmw3901_write_reg(0x70, 0x00);
    if (error == ESP_OK) {
        error = pmw3901_write_reg(PMW3901_REG_FRAME_STATUS, 0xFF);
    }
    if (error != ESP_OK) {
        return error;
    }

    return pmw3901_frame_wait_ready();
}
