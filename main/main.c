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

#define PKT_SIZE    37
#define PAY_SIZE    32

#define CMD_FLUSH_RX    0xE2
#define EN_CRC_BIT (1 << 3)   // bit 3 of CONFIG


#define CMD_W_REGISTER  0x20
#define CMD_R_REGISTER  0x00
#define REG_RX_ADDR_P0  0x0A
                        
#define REG_EN_AA       0x01       
#define REG_EN_RXADDR   0x02
#define REG_SETUP_AW    0x03
#define REG_RF_SETUP    0x06
#define REG_STATUS      0x07
#define REG_RX_PW_P0    0x11

spi_device_handle_t nrf;
uint64_t prom_addr = 0xAALL;
uint8_t channel = 25;
uint64_t addr;
uint8_t payload[PAY_SIZE];
uint8_t payload_size;
bool payload_encrypted = false;
uint8_t payload_type = 0;
uint16_t sequence;

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

void nrf_disable_crc(void)
{
    uint8_t config;
    nrf_read_register(CMD_R_REGISTER, &config, 1);
    config &= ~EN_CRC_BIT;
    nrf_write_register(CMD_R_REGISTER, &config, 1);
}

void nrf_start_listening(void)
{
    // 1. set PWR_UP (bit 1), PRIM_RX (bit 0) in CONFIG - rw
    uint8_t config;
    nrf_read_register(CMD_R_REGISTER, &config, 1);
    config |= (1 << 1) | (1 << 0); //PWR_UP | PRIM_RX
    nrf_write_register(CMD_R_REGISTER, &config, 1);

    // 2. flush the RX FIFO; this is SPI command byte, not a reg write
    uint8_t flush_cmd = CMD_FLUSH_RX;
    spi_transaction_t t = { .length = 8, .tx_buffer = &flush_cmd, .rx_buffer = NULL };
    spi_device_transmit(nrf, &t);

    // 3. clear any stale IRQ flags in STATUS (write 1 to clear RX_DR/TX_DS/MAX_RT)
    uint8_t clear_status = 0x70;
    nrf_write_register(REG_STATUS, &clear_status, 1);

    // 4. bring CE pin high; this is a plain GPIO write, not SPI at all.
    // CE physically switches the radio into active RX mode; everything above
    // just configures *what* it'll do once CE goes high
    gpio_set_level(PIN_CE, 1);

    // 5. datashee specifies ~130us Standby-I -> RX settling time
    vTaskDelay(pdMS_TO_TICKS(1));
}


void nrf_open_reading_pipe(int pipe, uint64_t prom_addr )
{
    uint8_t addr_bytes[3];
    addr_bytes[0] = (prom_addr >> 0)  & 0xFF;  // LSB first
    addr_bytes[1] = (prom_addr >> 8)  & 0xFF;
    addr_bytes[2] = (prom_addr >> 16) & 0xFF;
    // 1. Write the addr (3 bytes)
    nrf_write_register(REG_RX_ADDR_P0, addr_bytes, 3);

    // 2. Set expected payload width for pipe 0
    uint8_t payload_width = 32; // To be adjusted (default: 32)
    nrf_write_register(REG_RX_PW_P0, &payload_width, 1);

    // 3. Enable pipe 0 in EN_RXADDR (0x02) - rw
    uint8_t en_rxaddr;
    nrf_read_register(REG_EN_RXADDR, &en_rxaddr, 1);
    en_rxaddr |= (1 << pipe);
    nrf_write_register(REG_EN_RXADDR, &en_rxaddr, 1);
}

void scan(void)
{
    ESP_LOGI(TAG, "Starting scan...\n");

    int x, offset;
    uint8_t buf[PKT_SIZE];
    uint8_t wait = 100;
    uint8_t payload_len;
    uint16_t crc, crc_given;

    /********************************
     *  ORDER VERY IMPORTANT START  *
     ********************************/
    uint8_t en_aa_val = 0x00;
    uint8_t rf_setup_val = 0x09;
    uint8_t en_rxaddr_val = 0x00;
    uint8_t setup_aw_val = 0x00;

    nrf_write_register(REG_EN_AA, &en_aa_val, 1); // Disable auto ack on all(6) pipes
    nrf_write_register(REG_RF_SETUP, &rf_setup_val, 1); // Disable PA, 2M rate, LNA enabled
    

    nrf_write_register(REG_EN_RXADDR, &en_rxaddr_val, 1);
    nrf_write_register(REG_SETUP_AW, &setup_aw_val, 1);
    nrf_open_reading_pipe(0, prom_addr);
    nrf_start_listening();

    while(1)
    {
        channel++;
        if(channel > 84)
        {
            ESP_LOGI(TAG, "Starting channel sweep\n");
            channel = 2;
        }

        if(channel == 4)
        {
            ESP_LOGI(TAG, "LOW\n");
        }
        if(channel == 42)
        {
            ESP_LOGI(TAG, "HIGH\n");
        }
        if(channel == 44)
        {
            ESP_LOGI(TAG, "LOW\n");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{

    ESP_LOGI(TAG, "Starting %s...\n", TAG);
    nrf_spi_init();
    vTaskDelay(pdMS_TO_TICKS(200));
    uint8_t ret = nrf_read_status();
    ESP_LOGI(TAG, "read status %02x\n", ret);

    scan();
    return;
}
