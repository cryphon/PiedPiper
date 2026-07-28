#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

#define TAG "PiedPiper"

#define PIN_CE      4   // White
#define PIN_CSN     5   // Blue
#define PIN_SCK     18  // Yellow
#define PIN_MISO    19  // Green
#define PIN_MOSI    23  // Orange


#define CMD_W_REGISTER 0x20
#define CMD_R_REGISTER 0x00
#define REG_RX_ADDR_P0 0x0A

spi_device_handle_t nrf;

void nrf_spi_init(void)
{
    // 1. conf bus (shared clock/MISO/MOSI)
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_MISO,
        .mosi_io_num = PIN_MOSI,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    // 2. add nRF24 as device on that bus
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000, // 4MHz, safely under max 8
        .mode = 0,
        .spics_io_num = PIN_CSN,
        .queue_size = 1,
    };
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &nrf);
    ESP_ERROR_CHECK(ret);

    // 3. CE is separate from SPI; plain GPIO output
    gpio_set_direction(PIN_CE, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_CE, 0);
}

uint8_t nrf_read_status(void)
{
    uint8_t tx = 0xFF; // NOP
    uint8_t rx = 0;

    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &tx,
        .rx_buffer = &rx,
    };
    spi_device_transmit(nrf, &t);
    return rx;
}


void nrf_write_register(uint8_t reg, const uint8_t* data, size_t len)
{
    uint8_t tx[6] = {0};
    uint8_t rx[6] = {0};
    tx[0] = CMD_W_REGISTER | reg;
    memcpy(&tx[1], data, len);

    spi_transaction_t t = {
        .length = (len + 1) * 8, // bits not bytes
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    ESP_ERROR_CHECK(spi_device_transmit(nrf, &t));
}

void nrf_read_register(uint8_t reg, uint8_t *out, size_t len)
{
    uint8_t tx[6] = {0};
    uint8_t rx[6] = {0};
    tx[0] = CMD_R_REGISTER | reg;
    // remaining tx bytes stay 0x00 — they're just clocking, ignored by the chip

    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    ESP_ERROR_CHECK(spi_device_transmit(nrf, &t));
    memcpy(out, &rx[1], len); // skip byte 0 — that slot returned STATUS, not data
}



void app_main(void)
{

    ESP_LOGI(TAG, "Starting %s...\n", TAG);
    nrf_spi_init();
    vTaskDelay(pdMS_TO_TICKS(200));
    uint8_t ret = nrf_read_status();
    ESP_LOGI(TAG, "read status %02x\n", ret);


    uint8_t primer[5] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
    uint8_t readback[5] = {0};

    nrf_write_register(REG_RX_ADDR_P0, primer, 5);
    nrf_read_register(REG_RX_ADDR_P0, readback, 5);

    bool ok = memcmp(primer, readback, 5) == 0;
    ESP_LOGI(TAG, "RX_ADDR_P0 primer test: %s (got %02X %02X %02X %02X %02X)",
            ok ? "PASS" : "FAIL",
            readback[0], readback[1], readback[2], readback[3], readback[4]);
    return;
}
