#include <stdio.h>
#include "pico/stdlib.h"

#include "board_pins.h"

int main()
{
    stdio_init_all();
    board_pins_init();

    printf("Range test Started.\n");
    
}
