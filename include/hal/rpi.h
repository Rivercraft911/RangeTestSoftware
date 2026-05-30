#ifndef RPI_H
#define RPI_H

#include <stdbool.h>

// Raspberry Pi payload link helpers.
//
// Mirrors the samwise picubed RPi interface: a power-enable GPIO and a UART
// (see board_pins.h for the pin map). rpi_init() registers the RPi UART as an
// additional stdio driver, so the existing text command/data protocols (e.g.
// image transfer) work over the wired RPi link in addition to USB.

// Power on the Pi and bring up the RPi UART as a stdio driver.
void rpi_init(void);

// Drive the power-enable line (true = powered/on).
void rpi_set_enabled(bool on);

#endif
