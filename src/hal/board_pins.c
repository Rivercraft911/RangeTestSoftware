#include "hal/board_pins.h"

#include "pico/stdlib.h"

static void init_output_high(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 1);
}

static void init_output_low(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
}

static void init_input(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
}

void board_pins_init(void) {
    spi_init(UHF_SPI_PORT, RADIO_SPI_BAUD_HZ);
    spi_set_format(UHF_SPI_PORT, 8u, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PIN_UHF_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_UHF_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_UHF_MOSI, GPIO_FUNC_SPI);
    init_output_high(PIN_UHF_CS);
    init_output_high(PIN_UHF_RST);
    init_input(PIN_UHF_DIO0);

    spi_init(SBAND_SPI_PORT, RADIO_SPI_BAUD_HZ);
    spi_set_format(SBAND_SPI_PORT, 8u, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PIN_SBAND_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SBAND_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SBAND_MOSI, GPIO_FUNC_SPI);
    init_output_high(PIN_SBAND_CS);
    init_output_high(PIN_SBAND_RST);
    init_output_low(PIN_SBAND_TXEN);
    init_output_low(PIN_SBAND_RXEN);
    init_input(PIN_SBAND_BUSY);
    init_input(PIN_SBAND_DIO1);

    init_output_low(PIN_NEOPIXEL);
}
