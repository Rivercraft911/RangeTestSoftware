#include <stdio.h>

#include "hardware/gpio.h"
#include "pico/stdlib.h"

#define TEST_PIN 26u

int main(void) {
    stdio_init_all();

    gpio_init(TEST_PIN);
    gpio_set_dir(TEST_PIN, GPIO_OUT);
    gpio_put(TEST_PIN, 0);

    sleep_ms(1500);
    printf("USB sanity test started.\n");
    printf("If you can read this, USB serial is good.\n");

    bool high = false;
    while (true) {
        high = !high;
        gpio_put(TEST_PIN, high ? 1 : 0);
        printf("tick pin26=%d\n", high ? 1 : 0);
        sleep_ms(1000);
    }
}
