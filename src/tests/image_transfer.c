#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hal/board_pins.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "protocol/image_packet.h"
#include "radio/radio_sx1280f27.h"
#include "radio/radio_types.h"

#define SBAND_RF_PROFILE    1u
#define SBAND_TX_POWER_DBM  13
#define TX_TIMEOUT_MS       1000u
#define RX_TIMEOUT_MS       5000u
#define TX_INTER_PKT_MS     10u
#define BOOT_DELAY_MS       2000u

#define CMD_BUF_SIZE        600u

static char g_cmd_buf[CMD_BUF_SIZE];
static uint16_t g_cmd_pos = 0u;

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

// Non-blocking: try to read a complete line from serial.
// Returns true if g_cmd_buf contains a complete line.
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

// Blocking: wait up to timeout_ms for a complete line.
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
        tight_loop_contents();
    }
}

// ---------------------------------------------------------------------------
// TX session: serial -> LoRa
// ---------------------------------------------------------------------------

static void handle_tx_session(void) {
    uint16_t total_pkts = (uint16_t)parse_int_field(g_cmd_buf, "total_pkts=", 0);
    printf("IMG_ACK,ready\n");

    // Send START over LoRa
    image_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic = IMAGE_PACKET_MAGIC;
    pkt.pkt_type = IMAGE_PKT_START;
    pkt.total_pkts = total_pkts;
    pkt.data_len = 0u;
    send_lora_packet(&pkt);
    sleep_ms(TX_INTER_PKT_MS);

    // Read chunks from serial, send each over LoRa
    uint16_t sent = 0u;
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

        if (send_lora_packet(&pkt)) {
            printf("IMG_ACK,pkt=%u\n", (unsigned)pkt_num);
            sent++;
        } else {
            printf("IMG_NACK,pkt=%u,reason=tx_fail\n", (unsigned)pkt_num);
        }

        sleep_ms(TX_INTER_PKT_MS);
    }

    // Send END over LoRa
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic = IMAGE_PACKET_MAGIC;
    pkt.pkt_type = IMAGE_PKT_END;
    pkt.total_pkts = total_pkts;
    pkt.data_len = 0u;
    send_lora_packet(&pkt);

    printf("IMG_COMPLETE,sent=%u,total=%u\n", (unsigned)sent, (unsigned)total_pkts);
}

// ---------------------------------------------------------------------------
// RX: LoRa -> serial
// ---------------------------------------------------------------------------

static void handle_rx_packet(const radio_rx_frame_t *frame) {
    if (frame->length < IMAGE_HEADER_SIZE) return;

    image_packet_t pkt;
    memcpy(&pkt, frame->data, IMAGE_HEADER_SIZE);
    if (pkt.magic != IMAGE_PACKET_MAGIC) return;

    uint8_t data_bytes = frame->length - IMAGE_HEADER_SIZE;
    if (data_bytes > IMAGE_DATA_PER_PKT) data_bytes = IMAGE_DATA_PER_PKT;
    if (data_bytes > 0u) {
        memcpy(pkt.data, &frame->data[IMAGE_HEADER_SIZE], data_bytes);
    }

    switch (pkt.pkt_type) {
    case IMAGE_PKT_START:
        printf("IMG_START,total_pkts=%u\n", (unsigned)pkt.total_pkts);
        break;

    case IMAGE_PKT_DATA:
        printf("IMG_DATA,pkt=%u,len=%u,rssi=%d,snr=%d,hex=",
               (unsigned)pkt.pkt_num,
               (unsigned)pkt.data_len,
               (int)frame->rssi_dbm_x100,
               (int)frame->snr_db_x100);
        for (uint8_t i = 0u; i < pkt.data_len; ++i) {
            printf("%02X", pkt.data[i]);
        }
        printf("\n");
        break;

    case IMAGE_PKT_END:
        printf("IMG_END,total_pkts=%u\n", (unsigned)pkt.total_pkts);
        break;

    default:
        break;
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(void) {
    stdio_init_all();
    sleep_ms(BOOT_DELAY_MS);

    board_pins_init();

    radio_status_t st = sx1280f27_init();
    if (st != RADIO_STATUS_OK) {
        printf("FATAL,sband_init,st=%d\n", (int)st);
        while (true) sleep_ms(1000);
    }
    sx1280f27_set_profile(SBAND_RF_PROFILE);
    sx1280f27_set_tx_power_dbm(SBAND_TX_POWER_DBM);

    printf("ImageTransfer ready\n");

    // Arm RX
    sx1280f27_start_rx(RX_TIMEOUT_MS);

    while (true) {
        // Check serial for TX commands
        if (try_read_line()) {
            if (strncmp(g_cmd_buf, "IMG_SEND", 8) == 0) {
                sx1280f27_abort();
                handle_tx_session();
                // Return to RX mode
                sx1280f27_start_rx(RX_TIMEOUT_MS);
            }
        }

        // Poll LoRa
        radio_event_t ev = RADIO_EVENT_NONE;
        sx1280f27_poll_event(&ev);

        if (ev == RADIO_EVENT_RX_DONE) {
            radio_rx_frame_t frame;
            if (sx1280f27_read_rx(&frame) == RADIO_STATUS_OK) {
                handle_rx_packet(&frame);
            }
            sx1280f27_start_rx(RX_TIMEOUT_MS);
        } else if (ev == RADIO_EVENT_TIMEOUT) {
            sx1280f27_start_rx(RX_TIMEOUT_MS);
        } else if (ev == RADIO_EVENT_CRC_FAIL || ev == RADIO_EVENT_ERROR) {
            sx1280f27_abort();
            sx1280f27_start_rx(RX_TIMEOUT_MS);
        }

        sleep_ms(1);
    }
}
