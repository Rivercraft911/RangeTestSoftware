#include <stdio.h>
#include "pico/stdlib.h"

#include "hal/board_pins.h"
#include "protocol/range_packet.h"


int main()
{
    stdio_init_all();
    board_pins_init();

    printf("Range test Started.\n");

}
