#include <stdio.h>
#include <string.h>

#include "hal/board_pins.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "protocol/range_packet.h"
#include "radio/radio_rfm98pw.h"
#include "radio/radio_sx1280f27.h"
#include "radio/radio_types.h"
#include "ws2812.pio.h"

typedef enum {
    TEST_RADIO_NONE = 0,
    TEST_RADIO_SBAND,
    TEST_RADIO_UHF
} test_radio_t;

static uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 8) | ((uint32_t)g << 16) | b;
}

static void put_pixel(PIO pio, uint sm, uint32_t pixel_grb) {
    pio_sm_put_blocking(pio, sm, pixel_grb << 8u);
}

static void set_led_color(PIO pio, uint sm, uint8_t r, uint8_t g, uint8_t b) {
    put_pixel(pio, sm, urgb_u32(r, g, b));
}

static const char *event_to_str(radio_event_t event) {
    switch (event) {
        case RADIO_EVENT_NONE:
            return "NONE";
        case RADIO_EVENT_TX_DONE:
            return "TX_DONE";
        case RADIO_EVENT_RX_DONE:
            return "RX_DONE";
        case RADIO_EVENT_TIMEOUT:
            return "TIMEOUT";
        case RADIO_EVENT_CRC_FAIL:
            return "CRC_FAIL";
        case RADIO_EVENT_ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
    }
}

static void log_status(const char *tag, radio_status_t st) {
    if (st != RADIO_STATUS_OK) {
        printf("[%s] status=%d\n", tag, (int)st);
    }
}

static test_radio_t init_any_radio(void) {
    radio_status_t st = sx1280f27_init();
    if (st == RADIO_STATUS_OK) {
        (void)sx1280f27_set_profile(1u);
        (void)sx1280f27_set_tx_power_dbm(10);
        printf("Using SX1280 S-band radio.\n");
        return TEST_RADIO_SBAND;
    }
    printf("SX1280 init failed (%d), trying RFM98...\n", (int)st);

    st = rfm98pw_init();
    if (st == RADIO_STATUS_OK) {
        (void)rfm98pw_set_profile(1u);
        (void)rfm98pw_set_tx_power_dbm(17);
        printf("Using RFM98 UHF radio.\n");
        return TEST_RADIO_UHF;
    }
    printf("RFM98 init failed (%d).\n", (int)st);
    return TEST_RADIO_NONE;
}

static radio_status_t radio_start_tx(test_radio_t radio, const uint8_t *payload, uint8_t len, uint32_t timeout_ms) {
    if (radio == TEST_RADIO_SBAND) {
        return sx1280f27_start_tx(payload, len, timeout_ms);
    }
    if (radio == TEST_RADIO_UHF) {
        return rfm98pw_start_tx(payload, len, timeout_ms);
    }
    return RADIO_STATUS_NOT_READY;
}

static radio_status_t radio_start_rx(test_radio_t radio, uint32_t timeout_ms) {
    if (radio == TEST_RADIO_SBAND) {
        return sx1280f27_start_rx(timeout_ms);
    }
    if (radio == TEST_RADIO_UHF) {
        return rfm98pw_start_rx(timeout_ms);
    }
    return RADIO_STATUS_NOT_READY;
}

static radio_status_t radio_poll_event(test_radio_t radio, radio_event_t *event) {
    if (radio == TEST_RADIO_SBAND) {
        return sx1280f27_poll_event(event);
    }
    if (radio == TEST_RADIO_UHF) {
        return rfm98pw_poll_event(event);
    }
    return RADIO_STATUS_NOT_READY;
}

static radio_status_t radio_read_rx(test_radio_t radio, radio_rx_frame_t *frame) {
    if (radio == TEST_RADIO_SBAND) {
        return sx1280f27_read_rx(frame);
    }
    if (radio == TEST_RADIO_UHF) {
        return rfm98pw_read_rx(frame);
    }
    return RADIO_STATUS_NOT_READY;
}

static void radio_abort(test_radio_t radio) {
    if (radio == TEST_RADIO_SBAND) {
        (void)sx1280f27_abort();
    } else if (radio == TEST_RADIO_UHF) {
        (void)rfm98pw_abort();
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(1200);
    printf("Manual radio bring-up test starting.\n");

    board_pins_init();

    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, PIN_NEOPIXEL, 800000.0f, false);
    set_led_color(pio, sm, 0u, 0u, 0u);

    test_radio_t radio = init_any_radio();
    if (radio == TEST_RADIO_NONE) {
        printf("No radio initialized. LED-only mode.\n");
    }

    uint32_t packet_counter = 0u;
    absolute_time_t phase_deadline = delayed_by_ms(get_absolute_time(), 3000);
    bool tx_phase = true;

    bool op_active = false;
    bool led_on = false;
    absolute_time_t led_next_toggle = get_absolute_time();

    while (true) {
        if (absolute_time_diff_us(get_absolute_time(), phase_deadline) <= 0) {
            tx_phase = !tx_phase;
            phase_deadline = delayed_by_ms(get_absolute_time(), 3000);
            op_active = false;
            radio_abort(radio);
            printf("---- Enter %s phase ----\n", tx_phase ? "TX" : "RX");
        }

        if (absolute_time_diff_us(get_absolute_time(), led_next_toggle) <= 0) {
            led_on = !led_on;
            if (tx_phase) {
                set_led_color(pio, sm, led_on ? 0u : 0u, led_on ? 0u : 0u, led_on ? 40u : 0u);
            } else {
                set_led_color(pio, sm, led_on ? 40u : 0u, led_on ? 0u : 0u, led_on ? 0u : 0u);
            }
            led_next_toggle = delayed_by_ms(get_absolute_time(), 250);
        }

        if (radio != TEST_RADIO_NONE) {
            if (!op_active) {
                if (tx_phase) {
                    range_packet_t pkt;
                    memset(&pkt, 0, sizeof(pkt));
                    pkt.version = RANGE_PACKET_VERSION;
                    pkt.msg_type = RANGE_MSG_TRANSMIT;
                    pkt.role = RANGE_ROLE_TRANSMITTER;
                    pkt.radio = (radio == TEST_RADIO_SBAND) ? RANGE_RADIO_SBAND : RANGE_RADIO_UHF;
                    pkt.test_id = 1u;
                    pkt.packet_number = packet_counter++;
                    pkt.uptime_ms = to_ms_since_boot(get_absolute_time());
                    pkt.tx_power_dbm = (radio == TEST_RADIO_SBAND) ? 10 : 17;
                    pkt.rf_profile = 1u;
                    pkt.local_rssi_dbm_x100 = RANGE_INVALID_RSSI_DBM_X100;
                    pkt.local_snr_db_x100 = RANGE_INVALID_SNR_DB_X100;
                    pkt.remote_rssi_dbm_x100 = RANGE_INVALID_RSSI_DBM_X100;
                    pkt.remote_snr_db_x100 = RANGE_INVALID_SNR_DB_X100;
                    pkt.payload_crc16 = 0u;

                    radio_status_t st = radio_start_tx(radio, (const uint8_t *)&pkt, (uint8_t)sizeof(pkt), 1000u);
                    log_status("start_tx", st);
                    op_active = (st == RADIO_STATUS_OK);
                } else {
                    radio_status_t st = radio_start_rx(radio, 250u);
                    log_status("start_rx", st);
                    op_active = (st == RADIO_STATUS_OK);
                }
            }

            radio_event_t event = RADIO_EVENT_NONE;
            radio_status_t st = radio_poll_event(radio, &event);
            if (st != RADIO_STATUS_OK) {
                printf("[poll] status=%d\n", (int)st);
                radio_abort(radio);
                op_active = false;
            } else if (event != RADIO_EVENT_NONE) {
                printf("[%s] event=%s\n", tx_phase ? "TX" : "RX", event_to_str(event));
                if (event == RADIO_EVENT_RX_DONE) {
                    radio_rx_frame_t frame;
                    radio_status_t rx_st = radio_read_rx(radio, &frame);
                    if (rx_st == RADIO_STATUS_OK) {
                        printf("RX len=%u rssi_x100=%d snr_x100=%d\n",
                               frame.length, frame.rssi_dbm_x100, frame.snr_db_x100);
                    } else {
                        printf("read_rx status=%d\n", (int)rx_st);
                    }
                }
                op_active = false;
            }
        }

        sleep_ms(5);
    }
}
