#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hal/board_pins.h"
#include "hal/rpi.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "protocol/image_packet.h"
#include "radio/radio_sx1280f27.h"
#include "radio/radio_types.h"
#include "ws2812.pio.h"

#define SBAND_RF_PROFILE    1u
#define SBAND_TX_POWER_DBM  13
#define TX_TIMEOUT_MS       1000u
#define RX_TIMEOUT_MS       5000u
#define BOOT_DELAY_MS       2000u
#define ARQ_RX_TIMEOUT_MS   8000u
#define ARQ_MAX_ROUNDS      3u
#define LED_UPDATE_INTERVAL 10u

#define CMD_BUF_SIZE        600u

static char g_cmd_buf[CMD_BUF_SIZE];
static uint16_t g_cmd_pos = 0u;

// ---------------------------------------------------------------------------
// LED (WS2812 NeoPixel)
// ---------------------------------------------------------------------------

static PIO g_led_pio;
static uint g_led_sm;

static void led_init(void) {
    g_led_pio = pio0;
    g_led_sm = pio_claim_unused_sm(g_led_pio, true);
    uint offset = pio_add_program(g_led_pio, &ws2812_program);
    ws2812_program_init(g_led_pio, g_led_sm, offset, PIN_NEOPIXEL, 800000.0f, false);
}

static void led_set(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t grb = ((uint32_t)r << 8) | ((uint32_t)g << 16) | b;
    pio_sm_put_blocking(g_led_pio, g_led_sm, grb << 8u);
}

static void led_off(void) { led_set(0, 0, 0); }
static void led_red(void) { led_set(40, 0, 0); }
static void led_blue(void) { led_set(0, 0, 40); }
static void led_green(void) { led_set(0, 40, 0); }

static void led_flash_green(void) {
    for (int i = 0; i < 3; i++) {
        led_green();
        sleep_ms(150);
        led_off();
        sleep_ms(150);
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static uint8_t hex_nibble(char c) {
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0u;
}

static uint16_t hex_decode(const char *hex, uint8_t *out, uint16_t max_len) {
    uint16_t i = 0u;
    while (hex[0] && hex[1] && i < max_len) {
        out[i++] = (uint8_t)((hex_nibble(hex[0]) << 4) | hex_nibble(hex[1]));
        hex += 2;
    }
    return i;
}

static const char *find_field(const char *line, const char *key) {
    const char *p = strstr(line, key);
    if (!p) return NULL;
    return p + strlen(key);
}

static int parse_int_field(const char *line, const char *key, int fallback) {
    const char *v = find_field(line, key);
    if (!v) return fallback;
    return atoi(v);
}

static bool try_read_line(void) {
    while (true) {
        int ch = getchar_timeout_us(0);
        if (ch == PICO_ERROR_TIMEOUT) return false;
        if (ch == '\n' || ch == '\r') {
            if (g_cmd_pos > 0u) {
                g_cmd_buf[g_cmd_pos] = '\0';
                g_cmd_pos = 0u;
                return true;
            }
            continue;
        }
        if (g_cmd_pos < CMD_BUF_SIZE - 1u) {
            g_cmd_buf[g_cmd_pos++] = (char)ch;
        }
    }
}

static bool wait_for_line(uint32_t timeout_ms) {
    uint64_t deadline = to_ms_since_boot(get_absolute_time()) + timeout_ms;
    while (to_ms_since_boot(get_absolute_time()) < deadline) {
        if (try_read_line()) return true;
        sleep_ms(1);
    }
    return false;
}

// ---------------------------------------------------------------------------
// Radio helpers
// ---------------------------------------------------------------------------

static bool send_lora_packet(const image_packet_t *pkt) {
    uint8_t tx_len = IMAGE_HEADER_SIZE + pkt->data_len;
    radio_status_t st = sx1280f27_start_tx((const uint8_t *)pkt, tx_len, TX_TIMEOUT_MS);
    if (st != RADIO_STATUS_OK) {
        printf("ERROR,start_tx,st=%d\n", (int)st);
        return false;
    }
    while (true) {
        radio_event_t ev = RADIO_EVENT_NONE;
        st = sx1280f27_poll_event(&ev);
        if (st != RADIO_STATUS_OK) {
            sx1280f27_abort();
            return false;
        }
        if (ev == RADIO_EVENT_TX_DONE) return true;
        if (ev == RADIO_EVENT_TIMEOUT || ev == RADIO_EVENT_ERROR) {
            sx1280f27_abort();
            return false;
        }
        sleep_us(100);
    }
}

static bool wait_for_lora_rx(radio_rx_frame_t *frame, uint32_t timeout_ms) {
    sx1280f27_start_rx(timeout_ms);
    uint64_t deadline = to_ms_since_boot(get_absolute_time()) + timeout_ms;
    while (to_ms_since_boot(get_absolute_time()) < deadline) {
        radio_event_t ev = RADIO_EVENT_NONE;
        sx1280f27_poll_event(&ev);
        if (ev == RADIO_EVENT_RX_DONE) {
            if (sx1280f27_read_rx(frame) == RADIO_STATUS_OK) {
                return true;
            }
        }
        if (ev == RADIO_EVENT_TIMEOUT) return false;
        if (ev == RADIO_EVENT_CRC_FAIL || ev == RADIO_EVENT_ERROR) {
            sx1280f27_abort();
            sx1280f27_start_rx(timeout_ms);
        }
        tight_loop_contents();
    }
    sx1280f27_abort();
    return false;
}

// ---------------------------------------------------------------------------
// TX session: serial -> LoRa (with ARQ)
// ---------------------------------------------------------------------------

static void handle_tx_session(void) {
    uint16_t total_pkts = (uint16_t)parse_int_field(g_cmd_buf, "total_pkts=", 0);
    printf("IMG_ACK,ready\n");

    led_blue();

    image_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic = IMAGE_PACKET_MAGIC;
    pkt.pkt_type = IMAGE_PKT_START;
    pkt.total_pkts = total_pkts;
    pkt.data_len = 0u;
    send_lora_packet(&pkt);

    uint16_t sent = 0u;
    uint16_t pkt_counter = 0u;
    while (true) {
        if (!wait_for_line(10000u)) {
            printf("ERROR,serial_timeout\n");
            break;
        }

        if (strncmp(g_cmd_buf, "IMG_DONE", 8) == 0) {
            break;
        }

        if (strncmp(g_cmd_buf, "IMG_CHUNK", 9) != 0) {
            continue;
        }

        uint16_t pkt_num = (uint16_t)parse_int_field(g_cmd_buf, "pkt=", 0);
        uint8_t data_len = (uint8_t)parse_int_field(g_cmd_buf, "len=", 0);

        const char *hex_str = find_field(g_cmd_buf, "hex=");
        if (!hex_str || data_len == 0u || data_len > IMAGE_DATA_PER_PKT) {
            printf("IMG_NACK,pkt=%u,reason=bad_chunk\n", (unsigned)pkt_num);
            continue;
        }

        pkt.pkt_type = IMAGE_PKT_DATA;
        pkt.pkt_num = pkt_num;
        pkt.total_pkts = total_pkts;
        pkt.data_len = data_len;
        hex_decode(hex_str, pkt.data, data_len);

        // Throttle LED updates
        pkt_counter++;
        if (pkt_counter % LED_UPDATE_INTERVAL == 0u) {
            led_blue();
        } else if (pkt_counter % LED_UPDATE_INTERVAL == (LED_UPDATE_INTERVAL / 2)) {
            led_off();
        }

        if (send_lora_packet(&pkt)) {
            printf("IMG_ACK,pkt=%u\n", (unsigned)pkt_num);
            sent++;
        } else {
            printf("IMG_NACK,pkt=%u,reason=tx_fail\n", (unsigned)pkt_num);
        }
    }

    // Send END packet
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic = IMAGE_PACKET_MAGIC;
    pkt.pkt_type = IMAGE_PKT_END;
    pkt.total_pkts = total_pkts;
    pkt.data_len = 0u;
    send_lora_packet(&pkt);

    printf("IMG_COMPLETE,sent=%u,total=%u\n", (unsigned)sent, (unsigned)total_pkts);

    // ARQ: wait for NACK from receiver
    for (uint8_t round = 0; round < ARQ_MAX_ROUNDS; round++) {
        printf("ARQ_WAIT,round=%u\n", (unsigned)(round + 1));

        radio_rx_frame_t frame;
        if (!wait_for_lora_rx(&frame, ARQ_RX_TIMEOUT_MS)) {
            printf("ARQ_DONE,reason=no_nack\n");
            break;
        }

        // Check if it's a NACK packet
        if (frame.length < IMAGE_HEADER_SIZE) continue;
        image_packet_t nack_pkt;
        memcpy(&nack_pkt, frame.data, IMAGE_HEADER_SIZE);
        if (nack_pkt.magic != IMAGE_PACKET_MAGIC || nack_pkt.pkt_type != IMAGE_PKT_NACK) {
            printf("ARQ_DONE,reason=not_nack\n");
            break;
        }

        uint8_t nack_bytes = frame.length - IMAGE_HEADER_SIZE;
        if (nack_bytes > IMAGE_DATA_PER_PKT) nack_bytes = IMAGE_DATA_PER_PKT;
        memcpy(nack_pkt.data, &frame.data[IMAGE_HEADER_SIZE], nack_bytes);

        uint16_t missing_count = nack_bytes / 2u;
        if (missing_count == 0u) {
            printf("ARQ_DONE,reason=no_missing\n");
            break;
        }

        // Print which packets are needed
        printf("RETX_REQ,count=%u,pkts=", (unsigned)missing_count);
        uint16_t *missing_list = (uint16_t *)nack_pkt.data;
        for (uint16_t i = 0; i < missing_count; i++) {
            if (i > 0) printf(",");
            printf("%u", (unsigned)missing_list[i]);
        }
        printf("\n");

        // Wait for Python to re-send those chunks
        uint16_t retx_sent = 0u;
        while (true) {
            if (!wait_for_line(10000u)) {
                printf("ERROR,retx_serial_timeout\n");
                break;
            }
            if (strncmp(g_cmd_buf, "RETX_DONE", 9) == 0) break;
            if (strncmp(g_cmd_buf, "IMG_CHUNK", 9) != 0) continue;

            uint16_t rpkt = (uint16_t)parse_int_field(g_cmd_buf, "pkt=", 0);
            uint8_t rlen = (uint8_t)parse_int_field(g_cmd_buf, "len=", 0);
            const char *rhex = find_field(g_cmd_buf, "hex=");
            if (!rhex || rlen == 0u || rlen > IMAGE_DATA_PER_PKT) {
                printf("IMG_NACK,pkt=%u,reason=bad_chunk\n", (unsigned)rpkt);
                continue;
            }

            pkt.pkt_type = IMAGE_PKT_DATA;
            pkt.pkt_num = rpkt;
            pkt.total_pkts = total_pkts;
            pkt.data_len = rlen;
            hex_decode(rhex, pkt.data, rlen);

            if (send_lora_packet(&pkt)) {
                printf("IMG_ACK,pkt=%u\n", (unsigned)rpkt);
                retx_sent++;
            } else {
                printf("IMG_NACK,pkt=%u,reason=tx_fail\n", (unsigned)rpkt);
            }
        }

        // Send another END
        memset(&pkt, 0, sizeof(pkt));
        pkt.magic = IMAGE_PACKET_MAGIC;
        pkt.pkt_type = IMAGE_PKT_END;
        pkt.total_pkts = total_pkts;
        pkt.data_len = 0u;
        send_lora_packet(&pkt);
        printf("ARQ_RETX_COMPLETE,round=%u,sent=%u\n", (unsigned)(round + 1), (unsigned)retx_sent);
    }

    led_flash_green();
    led_off();
}

// ---------------------------------------------------------------------------
// RX: LoRa -> serial (with NACK support)
// ---------------------------------------------------------------------------

static bool g_rx_active = false;

static void handle_rx_packet(const radio_rx_frame_t *frame) {
    if (frame->length < IMAGE_HEADER_SIZE) return;

    image_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    memcpy(&pkt, frame->data, IMAGE_HEADER_SIZE);
    if (pkt.magic != IMAGE_PACKET_MAGIC) return;

    uint8_t data_bytes = frame->length - IMAGE_HEADER_SIZE;
    if (data_bytes > IMAGE_DATA_PER_PKT) data_bytes = IMAGE_DATA_PER_PKT;
    if (data_bytes > 0u) {
        memcpy(pkt.data, &frame->data[IMAGE_HEADER_SIZE], data_bytes);
    }

    // Use the actual received length, not just the header's claimed length
    uint8_t print_len = (pkt.data_len <= data_bytes) ? pkt.data_len : data_bytes;

    static uint16_t rx_pkt_counter = 0u;

    switch (pkt.pkt_type) {
    case IMAGE_PKT_START:
        g_rx_active = true;
        rx_pkt_counter = 0u;
        printf("IMG_START,total_pkts=%u\n", (unsigned)pkt.total_pkts);
        break;

    case IMAGE_PKT_DATA:
        rx_pkt_counter++;
        if (rx_pkt_counter % LED_UPDATE_INTERVAL == 0u) {
            led_red();
        } else if (rx_pkt_counter % LED_UPDATE_INTERVAL == (LED_UPDATE_INTERVAL / 2)) {
            led_off();
        }
        printf("IMG_DATA,pkt=%u,len=%u,rssi=%d,snr=%d,hex=",
               (unsigned)pkt.pkt_num,
               (unsigned)print_len,
               (int)frame->rssi_dbm_x100,
               (int)frame->snr_db_x100);
        for (uint8_t i = 0u; i < print_len; ++i) {
            printf("%02X", pkt.data[i]);
        }
        printf("\n");
        break;

    case IMAGE_PKT_END:
        g_rx_active = false;
        printf("IMG_END,total_pkts=%u\n", (unsigned)pkt.total_pkts);
        led_flash_green();
        break;

    default:
        break;
    }
}

static void handle_nack_send(void) {
    // Parse: IMG_NACK_SEND,missing=0,5,12,...
    const char *list_str = find_field(g_cmd_buf, "missing=");
    if (!list_str) {
        printf("NACK_ERR,reason=no_list\n");
        return;
    }

    image_packet_t nack;
    memset(&nack, 0, sizeof(nack));
    nack.magic = IMAGE_PACKET_MAGIC;
    nack.pkt_type = IMAGE_PKT_NACK;

    uint16_t *pkt_nums = (uint16_t *)nack.data;
    uint16_t count = 0u;
    uint16_t max_nums = IMAGE_DATA_PER_PKT / 2u;

    const char *p = list_str;
    while (*p && count < max_nums) {
        pkt_nums[count++] = (uint16_t)atoi(p);
        while (*p && *p != ',') p++;
        if (*p == ',') p++;
    }

    nack.data_len = (uint8_t)(count * 2u);
    nack.pkt_num = count;

    sx1280f27_abort();
    if (send_lora_packet(&nack)) {
        printf("NACK_SENT,count=%u\n", (unsigned)count);
    } else {
        printf("NACK_ERR,reason=tx_fail\n");
    }

    // Back to RX
    sx1280f27_start_rx(RX_TIMEOUT_MS);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(void) {
    stdio_init_all();
    sleep_ms(BOOT_DELAY_MS);

    board_pins_init();
    led_init();

    // Power the Raspberry Pi and add its UART as a stdio transport, so image
    // transfer works over the wired RPi link as well as USB.
    rpi_init();

    radio_status_t st = sx1280f27_init();
    if (st != RADIO_STATUS_OK) {
        printf("FATAL,sband_init,st=%d\n", (int)st);
        while (true) {
            led_red();
            sleep_ms(200);
            led_off();
            sleep_ms(200);
        }
    }
    sx1280f27_set_profile(SBAND_RF_PROFILE);
    sx1280f27_set_tx_power_dbm(SBAND_TX_POWER_DBM);

    printf("ImageTransfer ready\n");

    // Solid red = RX idle
    led_red();
    sx1280f27_start_rx(RX_TIMEOUT_MS);

    while (true) {
        if (try_read_line()) {
            if (strncmp(g_cmd_buf, "IMG_SEND", 8) == 0) {
                sx1280f27_abort();
                handle_tx_session();
                led_red();
                sx1280f27_start_rx(RX_TIMEOUT_MS);
            } else if (strncmp(g_cmd_buf, "SET_PROFILE", 11) == 0) {
                int prof = parse_int_field(g_cmd_buf, "profile=", 0);
                if (prof >= 1 && prof <= 4) {
                    sx1280f27_abort();
                    radio_status_t pst = sx1280f27_set_profile((uint8_t)prof);
                    if (pst == RADIO_STATUS_OK) {
                        printf("PROFILE_SET,profile=%d\n", prof);
                    } else {
                        printf("PROFILE_ERR,st=%d\n", (int)pst);
                    }
                    sx1280f27_start_rx(RX_TIMEOUT_MS);
                } else {
                    printf("PROFILE_ERR,reason=invalid\n");
                }
            } else if (strncmp(g_cmd_buf, "IMG_NACK_SEND", 13) == 0) {
                handle_nack_send();
            }
        }

        radio_event_t ev = RADIO_EVENT_NONE;
        sx1280f27_poll_event(&ev);

        if (ev == RADIO_EVENT_RX_DONE) {
            radio_rx_frame_t frame;
            if (sx1280f27_read_rx(&frame) == RADIO_STATUS_OK) {
                handle_rx_packet(&frame);
            }
            if (!g_rx_active) led_red();
            sx1280f27_start_rx(RX_TIMEOUT_MS);
        } else if (ev == RADIO_EVENT_TIMEOUT) {
            if (!g_rx_active) led_red();
            sx1280f27_start_rx(RX_TIMEOUT_MS);
        } else if (ev == RADIO_EVENT_CRC_FAIL || ev == RADIO_EVENT_ERROR) {
            sx1280f27_abort();
            sx1280f27_start_rx(RX_TIMEOUT_MS);
        }

        sleep_ms(1);
    }
}
