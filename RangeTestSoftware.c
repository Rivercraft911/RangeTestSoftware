#include <stdio.h>
#include "pico/stdlib.h"

#include "hal/board_pins.h"
#include "protocol/range_packet.h"
#include "radio/radio_rfm98pw.h"
#include "radio/radio_sx1280f27.h"


int main()
{
    stdio_init_all();
    board_pins_init();
    sx1280f27_init();
    rfm98pw_init();

    printf("Range test has begun, prepare thyself.\n");



    // // Main State Machine Loop 

    // while (1) {
        
    //     switch () {
    //         case();

    //         case();

    //         case();
    //     }
    // }


    return 0;
}
