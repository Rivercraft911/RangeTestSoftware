#include "hal/rpi.h"

#include "hal/board_pins.h"
#include "pico/stdio_uart.h"
#include "pico/stdlib.h"

void rpi_set_enabled(bool on) {
    gpio_put(PIN_RPI_ENABLE, on ? 1u : 0u);
}

void rpi_init(void) {
    // Power-enable line to the Raspberry Pi (mirrors samwise RPI_ENAB).
    gpio_init(PIN_RPI_ENABLE);
    gpio_set_dir(PIN_RPI_ENABLE, GPIO_OUT);
    rpi_set_enabled(true);

    // Bring up the RPi UART and register it as an additional stdio driver.
    // printf/getchar then fan out to both USB and the RPi link, so the
    // existing line-based protocols work transparently over either transport.
    stdio_uart_init_full(RPI_UART_PORT, RPI_UART_BAUD_HZ, PIN_RPI_TX, PIN_RPI_RX);
}
