#include <stdio.h>
#include <string.h>

#include "hal/board_pins.h"
#include "pico/stdlib.h"
#include "radio/radio_rfm98pw.h"
#include "radio/radio_types.h"

#define BOOT_DELAY_MS (10000u)
#define UHF_TX_POWER_DBM (20)
#define TX_TIMEOUT_MS (1000u)
#define TX_REARM_DELAY_MS (40u)
#define PAYLOAD_LEN (32u)

static void build_payload(uint8_t *payload, uint8_t len, uint32_t seq) {
    memset(payload, 0, len);
    payload[0] = 0x55u;
    payload[1] = 0xAAu;
    payload[2] = (uint8_t)(seq & 0xFFu);
    payload[3] = (uint8_t)((seq >> 8) & 0xFFu);
    payload[4] = (uint8_t)((seq >> 16) & 0xFFu);
    payload[5] = (uint8_t)((seq >> 24) & 0xFFu);
    for (uint8_t i = 6u; i < len; ++i) {
        payload[i] = (uint8_t)(i ^ 0xA5u);
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(1200);

    board_pins_init();

    printf("UHFConstantFullPowerTx booted. Waiting %u ms before TX start...\n", (unsigned)BOOT_DELAY_MS);
    sleep_ms(BOOT_DELAY_MS);

    radio_status_t st = rfm98pw_init();
    if (st != RADIO_STATUS_OK) {
        printf("rfm98 init failed st=%d\n", (int)st);
        while (true) {
            sleep_ms(100);
        }
    }

    st = rfm98pw_set_profile(1u);
    if (st != RADIO_STATUS_OK) {
        printf("rfm98 set_profile failed st=%d\n", (int)st);
        while (true) {
            sleep_ms(100);
        }
    }

    st = rfm98pw_set_tx_power_dbm(UHF_TX_POWER_DBM);
    if (st != RADIO_STATUS_OK) {
        printf("rfm98 set_tx_power failed st=%d\n", (int)st);
        while (true) {
            sleep_ms(100);
        }
    }

    printf("TX started: UHF full power=%d dBm, duty-limited mode.\n", UHF_TX_POWER_DBM);

    uint32_t seq = 0u;
    uint8_t payload[PAYLOAD_LEN];

    while (true) {
        build_payload(payload, PAYLOAD_LEN, seq);

        st = rfm98pw_start_tx(payload, PAYLOAD_LEN, TX_TIMEOUT_MS);
        if (st != RADIO_STATUS_OK) {
            printf("start_tx failed st=%d\n", (int)st);
            (void)rfm98pw_abort();
            sleep_ms(5);
            continue;
        }

        bool tx_done = false;
        while (!tx_done) {
            radio_event_t ev = RADIO_EVENT_NONE;
            st = rfm98pw_poll_event(&ev);
            if (st != RADIO_STATUS_OK) {
                printf("poll failed st=%d\n", (int)st);
                (void)rfm98pw_abort();
                break;
            }

            if (ev == RADIO_EVENT_TX_DONE) {
                tx_done = true;
            } else if ((ev == RADIO_EVENT_TIMEOUT) || (ev == RADIO_EVENT_ERROR)) {
                printf("tx event=%d\n", (int)ev);
                (void)rfm98pw_abort();
                break;
            }

            tight_loop_contents();
        }

        ++seq;
        if ((seq % 100u) == 0u) {
            printf("tx_count=%lu\n", (unsigned long)seq);
        }
        sleep_ms(TX_REARM_DELAY_MS);
    }
}
