#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#include "hardware/spi.h"
#include "hardware/uart.h"

// RPI & ACDS
// Raspberry Pi payload link. Pins mirror the samwise picubed board
// (uart0 on GPIO30/31, power-enable on GPIO29). NOTE: GPIO30/31 only exist on
// the RP2350B (picubed); on a stock pico2 (RP2350A) only GPIO0-29 are present.
#define RPI_UART_PORT uart0
#define RPI_UART_BAUD_HZ 115200u
#define PIN_RPI_TX 31u     // SAMWISE_UART_TX
#define PIN_RPI_RX 30u     // SAMWISE_UART_RX
#define PIN_RPI_ENABLE 29u // SAMWISE_RPI_ENAB
#define PIN_ACDS_TX 4u
#define PIN_ACDS_RX 5u

// MISC
#define PIN_MISC_6 6u
#define PIN_MISC_7 7u
#define PIN_NEOPIXEL 26u

// S-Band radio (LORA1280F27-TCXO)
#define SBAND_SPI_PORT spi1
#define PIN_SBAND_RST 8u
#define PIN_SBAND_BUSY 9u
#define PIN_SBAND_TXEN 10u
#define PIN_SBAND_RXEN 11u
#define PIN_SBAND_MISO 12u
#define PIN_SBAND_CS 13u
#define PIN_SBAND_SCK 14u
#define PIN_SBAND_MOSI 15u
#define PIN_SBAND_DIO1 20u

// UHF radio (RFM98PW)
#define UHF_SPI_PORT spi0
#define PIN_UHF_MISO 16u
#define PIN_UHF_CS 17u
#define PIN_UHF_SCK 18u
#define PIN_UHF_MOSI 19u
#define PIN_UHF_RST 21u
#define PIN_UHF_DIO0 28u

#define UHF_SPI_BAUD_HZ 1000000u
#define SBAND_SPI_BAUD_HZ 8000000u

void board_pins_init(void);

#endif
