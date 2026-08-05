#include "pmw3901_driver.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "PMW3901";

// ---------------------------------------------------------------------------
// TODO(pins): CONFIRM AGAINST YOUR WIRING.
// Already taken elsewhere: GPIO 11/12 (IMU I2C), GPIO 4/5/15/16 (motors),
// GPIO 17 (VL53L1X XSHUT), GPIO 6/7 (telemetry UART, see main.c).
// Avoid the strapping pins (0, 3, 45, 46) and the USB-JTAG pins (19, 20).
// ---------------------------------------------------------------------------
#define PMW3901_SPI_HOST SPI2_HOST
#define PMW3901_PIN_SCLK 18
#define PMW3901_PIN_MISO 8
#define PMW3901_PIN_MOSI 9
#define PMW3901_PIN_CS   10

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
// TODO(datasheet): VERIFY THESE TIMING DELAYS against the PMW3901 datasheet.
// PixArt specify minimum gaps between SPI transactions; violating them produces silent data
// corruption rather than an obvious failure, which is miserable to debug. The values below
// follow the widely used Bitcraze implementation but were not verified against a datasheet.
//   tSWW  - between two consecutive writes
//   tSWR  - between a write and a following read
//   tSRAD - between sending a read address and clocking the data byte out
//   tSRR/tSRW - after a read, before the next transaction
// ---------------------------------------------------------------------------
#define PMW3901_TIMING_TSWW_US  45
#define PMW3901_TIMING_TSWR_US  45
#define PMW3901_TIMING_TSRAD_US 35
#define PMW3901_TIMING_TSRR_US  20

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
static void pmw3901_cs_low(void) {
    gpio_set_level(PMW3901_PIN_CS, 0);
    esp_rom_delay_us(1);
}

static void pmw3901_cs_high(void) {
    esp_rom_delay_us(1);
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
    pmw3901_cs_high();

    esp_rom_delay_us(PMW3901_TIMING_TSWW_US);

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
    }

    pmw3901_cs_high();
    esp_rom_delay_us(PMW3901_TIMING_TSRR_US);

    if (error != ESP_OK) {
        ESP_LOGE(TAG, "SPI read of reg 0x%02x failed: %s", reg, esp_err_to_name(error));
        return error;
    }

    *value = rx;
    return ESP_OK;
}

// ===========================================================================
// STUB - YOU NEED TO FILL THIS IN.
//
// PixArt's PMW3901 requires roughly 80 register writes after power-up to configure its
// internal analogue front end and frame processing. The values are undocumented magic
// numbers from a PixArt application note; there is no way to derive them and writing
// plausible-looking guesses would produce a sensor that either returns nothing or returns
// convincing garbage.
//
// Transcribe the sequence from Bitcraze's driver, which is the reference implementation
// everyone uses and is MIT licensed:
//
//     https://github.com/bitcraze/Bitcraze_PMW3901
//     src/Bitcraze_PMW3901.cpp  ->  Bitcraze_PMW3901::initRegisters()
//
// It is a long flat list of registerWrite(addr, value) calls, including two timed pauses.
// Translate each one to:
//
//     error = pmw3901_write_reg(0xXX, 0xYY);
//     if (error != ESP_OK) return error;
//
// and keep the delays where Bitcraze have them. When you are done, delete the
// ESP_ERR_NOT_SUPPORTED return at the bottom.
// ===========================================================================
static esp_err_t pmw3901_write_init_sequence(void) {
    ESP_LOGE(TAG, "PMW3901 init register sequence has not been transcribed yet.");
    ESP_LOGE(TAG, "See pmw3901_write_init_sequence() in pmw3901_driver.c and copy the");
    ESP_LOGE(TAG, "sequence from https://github.com/bitcraze/Bitcraze_PMW3901");

    // <<< DELETE THIS RETURN once the register writes above are in place. >>>
    return ESP_ERR_NOT_SUPPORTED;
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

    // This is where it currently stops - see the stub above.
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
