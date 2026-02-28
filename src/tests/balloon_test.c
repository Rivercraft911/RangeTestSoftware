/*
 * balloon_test.c — Balloon range test firmware
 *
 * Single source file for both FLIGHT and GROUND roles.
 * Role is selected at compile time via BALLOON_FORCE_ROLE:
 *   1 = FLIGHT (autonomous balloon)
 *   2 = GROUND (USB-bridged ground station)
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hal/board_pins.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "protocol/balloon_packet.h"
#include "protocol/image_packet.h"
#include "radio/radio_rfm98pw.h"
#include "radio/radio_sx1280f27.h"
#include "radio/radio_types.h"
#include "ws2812.pio.h"

#include "data/balloon_image.h"
#include "data/balloon_text.h"

/* ------------------------------------------------------------------ */
/* Roles                                                              */
/* ------------------------------------------------------------------ */

enum { ROLE_FLIGHT = 1, ROLE_GROUND = 2 };

#ifndef BALLOON_FORCE_ROLE
#define BALLOON_FORCE_ROLE 0
#endif

/* ------------------------------------------------------------------ */
/* Timing constants                                                   */
/* ------------------------------------------------------------------ */

#define BOOT_DELAY_MS           2000u
#define BEACON_PERIOD_MS        2500u
#define RX_WINDOW_MS            2000u
#define TX_TIMEOUT_MS           1000u
#define RX_REARM_TIMEOUT_MS     250u
#define MAIN_LOOP_SLEEP_MS      2u
#define CMD_BUF_SIZE            600u

#define SBAND_DEFAULT_PROFILE   1u
#define SBAND_DEFAULT_POWER     13
#define UHF_PROFILE             1u
#define UHF_POWER_DBM           17

#define IMAGE_TX_DELAY_MS       15u
#define BULK_TX_DELAY_MS        10u
#define BULK_PKTS_PER_ROUND     200u

/* Max UHF payload must leave room for callsign within 255-byte RFM98PW limit */
#define UHF_MAX_PAYLOAD         (255u - CALLSIGN_LEN)

#define ARQ_MAX_ROUNDS          3u
#define ARQ_LISTEN_TIMEOUT_MS   5000u
#define STOP_CHECK_INTERVAL     20u

#define SWEEP_PKTS_PER_LEVEL    10u
#define SWEEP_STEP_DB           2

/* ------------------------------------------------------------------ */
/* Band abstraction (same pattern as range_test_node.c)               */
/* ------------------------------------------------------------------ */

typedef enum {
    BAND_SBAND = 2u,
    BAND_UHF   = 1u
} band_t;

typedef struct {
    uint8_t length;
    uint8_t data[255];
    int16_t rssi_dbm_x100;
    int16_t snr_db_x100;
} raw_frame_t;

static bool g_sband_ok = false;
static bool g_uhf_ok   = false;

static const char *band_str(band_t band) {
    return (band == BAND_SBAND) ? "SBAND" : "UHF";
}

static radio_status_t band_init(band_t band) {
    return (band == BAND_SBAND) ? sx1280f27_init() : rfm98pw_init();
}

static radio_status_t band_set_profile(band_t band, uint8_t profile_id) {
    return (band == BAND_SBAND) ? sx1280f27_set_profile(profile_id)
                                : rfm98pw_set_profile(profile_id);
}

static radio_status_t band_set_power(band_t band, int8_t dbm) {
    return (band == BAND_SBAND) ? sx1280f27_set_tx_power_dbm(dbm)
                                : rfm98pw_set_tx_power_dbm(dbm);
}

static radio_status_t band_start_tx(band_t band, const uint8_t *payload,
                                     uint8_t len, uint32_t timeout_ms) {
    return (band == BAND_SBAND)
        ? sx1280f27_start_tx(payload, len, timeout_ms)
        : rfm98pw_start_tx(payload, len, timeout_ms);
}

static radio_status_t band_start_rx(band_t band, uint32_t timeout_ms) {
    return (band == BAND_SBAND) ? sx1280f27_start_rx(timeout_ms)
                                : rfm98pw_start_rx(timeout_ms);
}

static radio_status_t band_poll_event(band_t band, radio_event_t *event) {
    return (band == BAND_SBAND) ? sx1280f27_poll_event(event)
                                : rfm98pw_poll_event(event);
}

static radio_status_t band_abort(band_t band) {
    return (band == BAND_SBAND) ? sx1280f27_abort() : rfm98pw_abort();
}

static radio_status_t band_read_frame(band_t band, raw_frame_t *out) {
    if (out == NULL) return RADIO_STATUS_INVALID_ARG;
    radio_rx_frame_t frame;
    radio_status_t st = (band == BAND_SBAND) ? sx1280f27_read_rx(&frame)
                                             : rfm98pw_read_rx(&frame);
    if (st != RADIO_STATUS_OK) return st;
    if (frame.length > sizeof(out->data)) return RADIO_STATUS_INTERNAL_ERROR;
    out->length = frame.length;
    out->rssi_dbm_x100 = frame.rssi_dbm_x100;
    out->snr_db_x100 = frame.snr_db_x100;
    memcpy(out->data, frame.data, frame.length);
    return RADIO_STATUS_OK;
}

/* ------------------------------------------------------------------ */
/* LED (WS2812 NeoPixel)                                              */
/* ------------------------------------------------------------------ */

static PIO g_led_pio;
static uint g_led_sm;

static void led_init(void) {
    g_led_pio = pio0;
    g_led_sm = pio_claim_unused_sm(g_led_pio, true);
    uint offset = pio_add_program(g_led_pio, &ws2812_program);
    ws2812_program_init(g_led_pio, g_led_sm, offset, PIN_NEOPIXEL,
                        800000.0f, false);
}

static void led_set(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t grb = ((uint32_t)r << 8) | ((uint32_t)g << 16) | b;
    pio_sm_put_blocking(g_led_pio, g_led_sm, grb << 8u);
}

static void led_off(void)    { led_set(0, 0, 0); }
static void led_green(void)  { led_set(0, 40, 0); }
static void led_blue(void)   { led_set(0, 0, 42); }
static void led_yellow(void) { led_set(34, 34, 0); }
static void led_red(void)    { led_set(36, 0, 0); }
static void led_purple(void) { led_set(30, 0, 34); }

static void led_orange_blink(void) {
    static bool on = false;
    on = !on;
    if (on) led_set(48, 18, 0);
    else    led_off();
}

/* ------------------------------------------------------------------ */
/* USB serial helpers (same pattern as image_transfer.c)              */
/* ------------------------------------------------------------------ */

static char g_cmd_buf[CMD_BUF_SIZE];
static uint16_t g_cmd_pos = 0u;

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

static void hex_encode(const uint8_t *data, uint16_t len, char *out) {
    static const char hex_chars[] = "0123456789abcdef";
    for (uint16_t i = 0; i < len; i++) {
        out[i * 2]     = hex_chars[data[i] >> 4];
        out[i * 2 + 1] = hex_chars[data[i] & 0x0f];
    }
    out[len * 2] = '\0';
}

/* ------------------------------------------------------------------ */
/* Blocking TX helpers                                                */
/* ------------------------------------------------------------------ */

static bool send_packet_blocking(band_t band, const void *packet,
                                  uint8_t len, uint32_t timeout_ms) {
    if ((packet == NULL) || (len == 0u)) return false;

    radio_status_t st = band_start_tx(band, (const uint8_t *)packet,
                                       len, timeout_ms);
    if (st != RADIO_STATUS_OK) {
        printf("[balloon] tx start failed band=%s st=%d\n",
               band_str(band), (int)st);
        led_orange_blink();
        return false;
    }

    uint64_t deadline = to_ms_since_boot(get_absolute_time()) + timeout_ms + 100u;
    while (to_ms_since_boot(get_absolute_time()) < deadline) {
        radio_event_t ev = RADIO_EVENT_NONE;
        st = band_poll_event(band, &ev);
        if (st != RADIO_STATUS_OK) {
            (void)band_abort(band);
            led_orange_blink();
            return false;
        }
        if (ev == RADIO_EVENT_TX_DONE) return true;
        if ((ev == RADIO_EVENT_TIMEOUT) || (ev == RADIO_EVENT_ERROR)) {
            (void)band_abort(band);
            led_orange_blink();
            return false;
        }
        sleep_ms(1);
    }

    (void)band_abort(band);
    led_orange_blink();
    return false;
}

/*
 * UHF TX with FCC callsign appended.
 * Copies the struct payload into a temp buffer, appends CALLSIGN, then sends.
 */
static bool uhf_send_with_callsign(const void *packet, uint8_t struct_len) {
    uint8_t buf[255];
    uint16_t total = (uint16_t)struct_len + CALLSIGN_LEN;
    if (total > sizeof(buf)) return false;
    memcpy(buf, packet, struct_len);
    memcpy(buf + struct_len, CALLSIGN, CALLSIGN_LEN);
    led_yellow();
    bool ok = send_packet_blocking(BAND_UHF, buf, (uint8_t)total, TX_TIMEOUT_MS);
    led_off();
    return ok;
}

/*
 * Strip callsign trailer from a received UHF frame (in-place length adjust).
 * Returns false if frame is too short to contain callsign.
 */
static bool uhf_strip_callsign(raw_frame_t *frame) {
    if (frame->length < CALLSIGN_LEN) return false;
    if (memcmp(frame->data + frame->length - CALLSIGN_LEN,
               CALLSIGN, CALLSIGN_LEN) != 0) {
        return false;
    }
    frame->length -= CALLSIGN_LEN;
    return true;
}

/* ------------------------------------------------------------------ */
/* Dual RX poll (same pattern as range_test_node.c)                   */
/* ------------------------------------------------------------------ */

typedef struct {
    bool sband_armed;
    bool uhf_armed;
} dual_rx_ctx_t;

static void dual_rx_init(dual_rx_ctx_t *ctx) {
    ctx->sband_armed = false;
    ctx->uhf_armed = false;
}

static bool dual_rx_poll(dual_rx_ctx_t *ctx, band_t *rx_band,
                          raw_frame_t *out_frame) {
    if (g_sband_ok && !ctx->sband_armed) {
        if (band_start_rx(BAND_SBAND, RX_REARM_TIMEOUT_MS) == RADIO_STATUS_OK)
            ctx->sband_armed = true;
    }
    if (g_uhf_ok && !ctx->uhf_armed) {
        if (band_start_rx(BAND_UHF, RX_REARM_TIMEOUT_MS) == RADIO_STATUS_OK)
            ctx->uhf_armed = true;
    }

    if (ctx->sband_armed) {
        radio_event_t ev = RADIO_EVENT_NONE;
        radio_status_t st = band_poll_event(BAND_SBAND, &ev);
        if (st == RADIO_STATUS_OK) {
            if (ev == RADIO_EVENT_RX_DONE) {
                if (band_read_frame(BAND_SBAND, out_frame) == RADIO_STATUS_OK) {
                    *rx_band = BAND_SBAND;
                    ctx->sband_armed = false;
                    return true;
                }
                ctx->sband_armed = false;
            } else if ((ev == RADIO_EVENT_TIMEOUT) ||
                       (ev == RADIO_EVENT_CRC_FAIL) ||
                       (ev == RADIO_EVENT_ERROR)) {
                ctx->sband_armed = false;
            }
        } else {
            ctx->sband_armed = false;
        }
    }

    if (ctx->uhf_armed) {
        radio_event_t ev = RADIO_EVENT_NONE;
        radio_status_t st = band_poll_event(BAND_UHF, &ev);
        if (st == RADIO_STATUS_OK) {
            if (ev == RADIO_EVENT_RX_DONE) {
                if (band_read_frame(BAND_UHF, out_frame) == RADIO_STATUS_OK) {
                    *rx_band = BAND_UHF;
                    ctx->uhf_armed = false;
                    return true;
                }
                ctx->uhf_armed = false;
            } else if ((ev == RADIO_EVENT_TIMEOUT) ||
                       (ev == RADIO_EVENT_CRC_FAIL) ||
                       (ev == RADIO_EVENT_ERROR)) {
                ctx->uhf_armed = false;
            }
        } else {
            ctx->uhf_armed = false;
        }
    }

    return false;
}

/* Single-band blocking RX with timeout */
static bool rx_blocking(band_t band, raw_frame_t *out, uint32_t timeout_ms) {
    radio_status_t st = band_start_rx(band, timeout_ms);
    if (st != RADIO_STATUS_OK) return false;
    uint64_t deadline = to_ms_since_boot(get_absolute_time()) + timeout_ms + 50u;
    while (to_ms_since_boot(get_absolute_time()) < deadline) {
        radio_event_t ev = RADIO_EVENT_NONE;
        st = band_poll_event(band, &ev);
        if (st != RADIO_STATUS_OK) { (void)band_abort(band); return false; }
        if (ev == RADIO_EVENT_RX_DONE) {
            return band_read_frame(band, out) == RADIO_STATUS_OK;
        }
        if ((ev == RADIO_EVENT_TIMEOUT) || (ev == RADIO_EVENT_ERROR)) {
            (void)band_abort(band);
            return false;
        }
        sleep_ms(1);
    }
    (void)band_abort(band);
    return false;
}

/* ------------------------------------------------------------------ */
/* PRBS-15 generator for bulk test                                    */
/* ------------------------------------------------------------------ */

static uint16_t prbs15_next(uint16_t state) {
    uint16_t bit = ((state >> 14) ^ (state >> 13)) & 1u;
    return (uint16_t)((state << 1) | bit) & 0x7FFFu;
}

static void fill_prbs(uint8_t *buf, uint16_t len, uint16_t seed) {
    uint16_t state = seed ? seed : 1u;
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = (uint8_t)(state & 0xFF);
        state = prbs15_next(state);
    }
}

/* ------------------------------------------------------------------ */
/* FLIGHT role                                                        */
/* ------------------------------------------------------------------ */

#if (BALLOON_FORCE_ROLE == 1)

typedef struct {
    balloon_state_t state;
    uint8_t  seq;
    uint32_t tx_count;
    uint32_t rx_count;
    int16_t  last_cmd_rssi_x100;
    int16_t  last_cmd_snr_x100;
    uint8_t  sband_profile;
    int8_t   sband_power_dbm;
    bool     image_loaded;

    /* Bulk test state */
    band_t   bulk_band;
    bool     bulk_text;
    uint16_t bulk_seq;
    uint16_t bulk_seed;

    /* Power sweep state */
    int8_t   sweep_from;
    int8_t   sweep_to;
    int8_t   sweep_current;
    uint8_t  sweep_count;
} flight_state_t;

static flight_state_t g_fs;

static void flight_init_state(void) {
    memset(&g_fs, 0, sizeof(g_fs));
    g_fs.state = BALLOON_STATE_BEACON;
    g_fs.sband_profile = SBAND_DEFAULT_PROFILE;
    g_fs.sband_power_dbm = SBAND_DEFAULT_POWER;
    g_fs.image_loaded = (BALLOON_IMAGE_SIZE > 0u);
    g_fs.last_cmd_rssi_x100 = INT16_MIN;
    g_fs.last_cmd_snr_x100 = INT16_MIN;
}

static void flight_send_beacon(band_t band) {
    balloon_beacon_t bcn;
    memset(&bcn, 0, sizeof(bcn));
    bcn.h.magic = BALLOON_MAGIC;
    bcn.h.msg_type = BALLOON_MSG_BEACON;
    bcn.h.seq = g_fs.seq++;
    bcn.uptime_ms = to_ms_since_boot(get_absolute_time());
    bcn.tx_count = g_fs.tx_count;
    bcn.rx_count = g_fs.rx_count;
    bcn.last_cmd_rssi_x100 = g_fs.last_cmd_rssi_x100;
    bcn.last_cmd_snr_x100 = g_fs.last_cmd_snr_x100;
    bcn.tx_power_dbm = (band == BAND_SBAND) ? g_fs.sband_power_dbm
                                             : (int8_t)UHF_POWER_DBM;
    bcn.sband_profile = g_fs.sband_profile;
    bcn.state = (uint8_t)g_fs.state;
    bcn.flags = 0u;
    if (g_fs.image_loaded) bcn.flags |= BALLOON_FLAG_IMAGE_LOADED;
    if (g_fs.state == BALLOON_STATE_BULK) bcn.flags |= BALLOON_FLAG_BULK_ACTIVE;

    if (band == BAND_UHF) {
        uhf_send_with_callsign(&bcn, sizeof(bcn));
    } else {
        led_blue();
        send_packet_blocking(BAND_SBAND, &bcn, sizeof(bcn), TX_TIMEOUT_MS);
        led_off();
    }
    g_fs.tx_count++;
}

static void flight_send_cmd_ack(const balloon_cmd_t *cmd,
                                 int16_t rssi_x100, int16_t snr_x100,
                                 uint8_t status) {
    balloon_cmd_ack_t ack;
    memset(&ack, 0, sizeof(ack));
    ack.h.magic = BALLOON_MAGIC;
    ack.h.msg_type = BALLOON_MSG_CMD_ACK;
    ack.h.seq = g_fs.seq++;
    ack.cmd_id = cmd->cmd_id;
    ack.ack_seq = cmd->h.seq;
    ack.status = status;
    ack.cmd_rssi_x100 = rssi_x100;
    ack.cmd_snr_x100 = snr_x100;
    uhf_send_with_callsign(&ack, sizeof(ack));
    g_fs.tx_count++;
}

static bool flight_check_uhf_stop(void) {
    raw_frame_t frame;
    if (!rx_blocking(BAND_UHF, &frame, 100u)) return false;
    if (!uhf_strip_callsign(&frame)) return false;
    if (frame.length < sizeof(balloon_cmd_t)) return false;
    const balloon_cmd_t *cmd = (const balloon_cmd_t *)frame.data;
    if (cmd->h.magic != BALLOON_MAGIC) return false;
    if (cmd->h.msg_type != BALLOON_MSG_CMD) return false;
    g_fs.rx_count++;
    g_fs.last_cmd_rssi_x100 = frame.rssi_dbm_x100;
    g_fs.last_cmd_snr_x100 = frame.snr_db_x100;
    flight_send_cmd_ack(cmd, frame.rssi_dbm_x100, frame.snr_db_x100, 0u);
    return (cmd->cmd_id == BALLOON_CMD_STOP);
}

static void flight_handle_command(const balloon_cmd_t *cmd,
                                   int16_t rssi_x100, int16_t snr_x100) {
    g_fs.rx_count++;
    g_fs.last_cmd_rssi_x100 = rssi_x100;
    g_fs.last_cmd_snr_x100 = snr_x100;

    uint8_t status = 0u;

    switch ((balloon_cmd_id_t)cmd->cmd_id) {
    case BALLOON_CMD_PING:
        break;

    case BALLOON_CMD_SET_SBAND_PROFILE:
        if (cmd->param1 >= 1u && cmd->param1 <= 4u) {
            g_fs.sband_profile = cmd->param1;
            (void)band_abort(BAND_SBAND);
            sleep_ms(2);
            (void)band_set_profile(BAND_SBAND, cmd->param1);
        } else {
            status = 1u;
        }
        break;

    case BALLOON_CMD_START_IMAGE:
        if (g_fs.image_loaded) {
            g_fs.state = BALLOON_STATE_IMAGE;
        } else {
            status = 2u;
        }
        break;

    case BALLOON_CMD_START_BULK:
        g_fs.bulk_band = (cmd->param1 == 2u) ? BAND_SBAND : BAND_UHF;
        g_fs.bulk_text = (cmd->param2 & 1u) != 0u;
        g_fs.bulk_seq = 0u;
        g_fs.bulk_seed = (uint16_t)(to_ms_since_boot(get_absolute_time()) & 0x7FFFu);
        if (g_fs.bulk_seed == 0u) g_fs.bulk_seed = 1u;
        g_fs.state = BALLOON_STATE_BULK;
        break;

    case BALLOON_CMD_STOP:
        g_fs.state = BALLOON_STATE_BEACON;
        break;

    case BALLOON_CMD_POWER_SWEEP:
        g_fs.sweep_from = (int8_t)cmd->param1;
        g_fs.sweep_to = (int8_t)cmd->param2;
        if (g_fs.sweep_from > g_fs.sweep_to) {
            int8_t tmp = g_fs.sweep_from;
            g_fs.sweep_from = g_fs.sweep_to;
            g_fs.sweep_to = tmp;
        }
        g_fs.sweep_current = g_fs.sweep_to;
        g_fs.sweep_count = 0u;
        g_fs.state = BALLOON_STATE_POWER_SWEEP;
        break;

    default:
        status = 0xFFu;
        break;
    }

    flight_send_cmd_ack(cmd, rssi_x100, snr_x100, status);
}

static void flight_run_image_downlink(void) {
    led_purple();

    uint16_t total_pkts = (uint16_t)((BALLOON_IMAGE_SIZE + IMAGE_DATA_PER_PKT - 1u)
                                      / IMAGE_DATA_PER_PKT);

    /* Send START packet */
    image_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic = IMAGE_PACKET_MAGIC;
    pkt.pkt_type = IMAGE_PKT_START;
    pkt.total_pkts = total_pkts;
    pkt.data_len = 0u;
    send_packet_blocking(BAND_SBAND, &pkt,
                         IMAGE_HEADER_SIZE + pkt.data_len, TX_TIMEOUT_MS);

    /* Blast all data packets */
    for (uint16_t i = 0; i < total_pkts; i++) {
        uint32_t offset = (uint32_t)i * IMAGE_DATA_PER_PKT;
        uint32_t remaining = BALLOON_IMAGE_SIZE - offset;
        uint8_t chunk = (remaining > IMAGE_DATA_PER_PKT)
                            ? (uint8_t)IMAGE_DATA_PER_PKT
                            : (uint8_t)remaining;

        memset(&pkt, 0, sizeof(pkt));
        pkt.magic = IMAGE_PACKET_MAGIC;
        pkt.pkt_type = IMAGE_PKT_DATA;
        pkt.pkt_num = i;
        pkt.total_pkts = total_pkts;
        pkt.data_len = chunk;
        memcpy(pkt.data, balloon_image + offset, chunk);

        led_blue();
        send_packet_blocking(BAND_SBAND, &pkt,
                             IMAGE_HEADER_SIZE + chunk, TX_TIMEOUT_MS);
        sleep_ms(IMAGE_TX_DELAY_MS);

        /* Periodically check for STOP command */
        if (((i + 1u) % STOP_CHECK_INTERVAL) == 0u) {
            if (flight_check_uhf_stop()) {
                g_fs.state = BALLOON_STATE_BEACON;
                led_green();
                return;
            }
        }
    }

    /* Send END packet */
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic = IMAGE_PACKET_MAGIC;
    pkt.pkt_type = IMAGE_PKT_END;
    pkt.total_pkts = total_pkts;
    pkt.data_len = 0u;
    send_packet_blocking(BAND_SBAND, &pkt,
                         IMAGE_HEADER_SIZE + pkt.data_len, TX_TIMEOUT_MS);

    /* ARQ: listen for NACK on UHF, retransmit on S-Band */
    for (uint8_t round = 0; round < ARQ_MAX_ROUNDS; round++) {
        raw_frame_t nack_frame;
        if (!rx_blocking(BAND_UHF, &nack_frame, ARQ_LISTEN_TIMEOUT_MS)) {
            break;  /* No NACK received — assume complete */
        }
        uhf_strip_callsign(&nack_frame);

        if (nack_frame.length < IMAGE_HEADER_SIZE) break;
        const image_packet_t *nack = (const image_packet_t *)nack_frame.data;
        if (nack->magic != IMAGE_PACKET_MAGIC) break;
        if (nack->pkt_type != IMAGE_PKT_NACK) break;

        /* NACK data contains list of 16-bit packet numbers */
        uint16_t nack_count = nack->data_len / 2u;
        for (uint16_t n = 0; n < nack_count; n++) {
            uint16_t pkt_num;
            memcpy(&pkt_num, nack->data + n * 2u, 2u);
            if (pkt_num >= total_pkts) continue;

            uint32_t off = (uint32_t)pkt_num * IMAGE_DATA_PER_PKT;
            uint32_t rem = BALLOON_IMAGE_SIZE - off;
            uint8_t chunk = (rem > IMAGE_DATA_PER_PKT)
                                ? (uint8_t)IMAGE_DATA_PER_PKT
                                : (uint8_t)rem;

            memset(&pkt, 0, sizeof(pkt));
            pkt.magic = IMAGE_PACKET_MAGIC;
            pkt.pkt_type = IMAGE_PKT_DATA;
            pkt.pkt_num = pkt_num;
            pkt.total_pkts = total_pkts;
            pkt.data_len = chunk;
            memcpy(pkt.data, balloon_image + off, chunk);
            send_packet_blocking(BAND_SBAND, &pkt,
                                 IMAGE_HEADER_SIZE + chunk, TX_TIMEOUT_MS);
            sleep_ms(IMAGE_TX_DELAY_MS);
        }

        /* Send another END after retransmissions */
        memset(&pkt, 0, sizeof(pkt));
        pkt.magic = IMAGE_PACKET_MAGIC;
        pkt.pkt_type = IMAGE_PKT_END;
        pkt.total_pkts = total_pkts;
        pkt.data_len = 0u;
        send_packet_blocking(BAND_SBAND, &pkt,
                             IMAGE_HEADER_SIZE + pkt.data_len, TX_TIMEOUT_MS);
    }

    g_fs.state = BALLOON_STATE_BEACON;
    led_green();
}

static void flight_run_bulk(void) {
    led_purple();

    for (uint16_t i = 0; i < BULK_PKTS_PER_ROUND; i++) {
        balloon_bulk_t bulk;
        memset(&bulk, 0, sizeof(bulk));
        bulk.h.magic = BALLOON_MAGIC;
        bulk.h.msg_type = BALLOON_MSG_BULK;
        bulk.h.seq = g_fs.seq++;
        bulk.seed = g_fs.bulk_seed;
        bulk.flags = g_fs.bulk_text ? BALLOON_BULK_FLAG_TEXT : 0u;

        /* Cap payload so UHF packets fit within 255 bytes with callsign */
        uint8_t max_data = (g_fs.bulk_band == BAND_UHF)
            ? (uint8_t)(UHF_MAX_PAYLOAD - BALLOON_BULK_HEADER_SIZE)
            : (uint8_t)BALLOON_BULK_MAX_PAYLOAD;

        if (g_fs.bulk_text) {
            uint8_t fill_len = (BALLOON_BULK_TEXT_LEN < max_data)
                                   ? BALLOON_BULK_TEXT_LEN : max_data;
            memcpy(bulk.data, g_balloon_bulk_text, fill_len);
            bulk.data_len = fill_len;
        } else {
            fill_prbs(bulk.data, max_data, g_fs.bulk_seed);
            bulk.data_len = max_data;
        }

        uint8_t tx_len = BALLOON_BULK_HEADER_SIZE + bulk.data_len;

        if (g_fs.bulk_band == BAND_UHF) {
            uhf_send_with_callsign(&bulk, tx_len);
        } else {
            led_blue();
            send_packet_blocking(BAND_SBAND, &bulk, tx_len, TX_TIMEOUT_MS);
        }

        g_fs.tx_count++;
        g_fs.bulk_seq++;
        g_fs.bulk_seed = prbs15_next(g_fs.bulk_seed);
        sleep_ms(BULK_TX_DELAY_MS);

        /* Periodically check for STOP */
        if (((i + 1u) % STOP_CHECK_INTERVAL) == 0u) {
            if (flight_check_uhf_stop()) {
                g_fs.state = BALLOON_STATE_BEACON;
                led_green();
                return;
            }
        }
    }

    g_fs.state = BALLOON_STATE_BEACON;
    led_green();
}

static void flight_run_power_sweep(void) {
    led_purple();

    for (int8_t pwr = g_fs.sweep_to; pwr >= g_fs.sweep_from;
         pwr -= SWEEP_STEP_DB) {
        (void)band_abort(BAND_SBAND);
        sleep_ms(2);
        (void)band_set_power(BAND_SBAND, pwr);
        g_fs.sband_power_dbm = pwr;

        for (uint8_t i = 0; i < SWEEP_PKTS_PER_LEVEL; i++) {
            flight_send_beacon(BAND_SBAND);
            sleep_ms(100);
        }

        /* Check for STOP */
        if (flight_check_uhf_stop()) {
            /* Restore max power */
            (void)band_set_power(BAND_SBAND, SBAND_DEFAULT_POWER);
            g_fs.sband_power_dbm = SBAND_DEFAULT_POWER;
            g_fs.state = BALLOON_STATE_BEACON;
            led_green();
            return;
        }
    }

    /* Restore max power */
    (void)band_set_power(BAND_SBAND, SBAND_DEFAULT_POWER);
    g_fs.sband_power_dbm = SBAND_DEFAULT_POWER;
    g_fs.state = BALLOON_STATE_BEACON;
    led_green();
}

static int run_flight(void) {
    printf("[balloon] FLIGHT role — starting\n");
    flight_init_state();

    while (true) {
        if (g_fs.state != BALLOON_STATE_BEACON) {
            switch (g_fs.state) {
            case BALLOON_STATE_IMAGE:
                flight_run_image_downlink();
                break;
            case BALLOON_STATE_BULK:
                flight_run_bulk();
                break;
            case BALLOON_STATE_POWER_SWEEP:
                flight_run_power_sweep();
                break;
            default:
                g_fs.state = BALLOON_STATE_BEACON;
                break;
            }
            continue;
        }

        /* --- Beacon cycle --- */

        /* 1. S-Band beacon */
        led_blue();
        flight_send_beacon(BAND_SBAND);
        led_off();

        /* 2. UHF beacon */
        led_yellow();
        flight_send_beacon(BAND_UHF);
        led_off();

        /* 3. UHF RX window for commands */
        led_red();
        uint64_t rx_deadline = to_ms_since_boot(get_absolute_time()) + RX_WINDOW_MS;
        radio_status_t st = band_start_rx(BAND_UHF, RX_WINDOW_MS);
        if (st != RADIO_STATUS_OK) {
            printf("[balloon] uhf rx start failed st=%d\n", (int)st);
            led_green();
            sleep_ms(BEACON_PERIOD_MS - RX_WINDOW_MS);
            continue;
        }

        bool got_cmd = false;
        while (to_ms_since_boot(get_absolute_time()) < rx_deadline) {
            radio_event_t ev = RADIO_EVENT_NONE;
            st = band_poll_event(BAND_UHF, &ev);
            if (st != RADIO_STATUS_OK) break;

            if (ev == RADIO_EVENT_RX_DONE) {
                raw_frame_t frame;
                if (band_read_frame(BAND_UHF, &frame) == RADIO_STATUS_OK) {
                    if (uhf_strip_callsign(&frame) &&
                        frame.length >= sizeof(balloon_cmd_t)) {
                        const balloon_cmd_t *cmd =
                            (const balloon_cmd_t *)frame.data;
                        if (cmd->h.magic == BALLOON_MAGIC &&
                            cmd->h.msg_type == BALLOON_MSG_CMD) {
                            flight_handle_command(cmd,
                                                   frame.rssi_dbm_x100,
                                                   frame.snr_db_x100);
                            got_cmd = true;
                            break;
                        }
                    }
                }
                /* Re-arm RX for remainder of window */
                uint64_t now = to_ms_since_boot(get_absolute_time());
                if (now < rx_deadline) {
                    (void)band_start_rx(BAND_UHF,
                                        (uint32_t)(rx_deadline - now));
                }
            } else if ((ev == RADIO_EVENT_TIMEOUT) ||
                       (ev == RADIO_EVENT_ERROR)) {
                break;
            }
            sleep_ms(1);
        }

        (void)band_abort(BAND_UHF);

        if (!got_cmd) {
            led_green();
        }
    }
}

#endif /* BALLOON_FORCE_ROLE == 1 */

/* ------------------------------------------------------------------ */
/* GROUND role                                                        */
/* ------------------------------------------------------------------ */

#if (BALLOON_FORCE_ROLE == 2)

static uint8_t g_ground_sband_profile = SBAND_DEFAULT_PROFILE;
static uint8_t g_ground_cmd_seq = 0u;
static uint8_t g_ground_pending_sband_profile = 0u;

static void ground_print_beacon(band_t rx_band, const balloon_beacon_t *bcn,
                                 int16_t rssi_x100, int16_t snr_x100) {
    printf("BEACON,band=%s,seq=%u,uptime_ms=%" PRIu32 ","
           "tx_count=%" PRIu32 ",rx_count=%" PRIu32 ","
           "last_cmd_rssi_x100=%d,last_cmd_snr_x100=%d,"
           "tx_power_dbm=%d,sband_profile=%u,"
           "state=%u,flags=0x%02x,"
           "rssi_x100=%d,snr_x100=%d\n",
           band_str(rx_band), bcn->h.seq, bcn->uptime_ms,
           bcn->tx_count, bcn->rx_count,
           (int)bcn->last_cmd_rssi_x100, (int)bcn->last_cmd_snr_x100,
           (int)bcn->tx_power_dbm, bcn->sband_profile,
           bcn->state, bcn->flags,
           (int)rssi_x100, (int)snr_x100);
}

static void ground_print_cmd_ack(const balloon_cmd_ack_t *ack,
                                  int16_t rssi_x100, int16_t snr_x100) {
    if (ack->cmd_id == BALLOON_CMD_SET_SBAND_PROFILE &&
        ack->status == 0u &&
        g_ground_pending_sband_profile >= 1u &&
        g_ground_pending_sband_profile <= 4u) {
        (void)band_abort(BAND_SBAND);
        sleep_ms(2);
        (void)band_set_profile(BAND_SBAND, g_ground_pending_sband_profile);
        g_ground_sband_profile = g_ground_pending_sband_profile;
        printf("PROFILE_SET,profile=%u\n", g_ground_pending_sband_profile);
    }
    if (ack->cmd_id == BALLOON_CMD_SET_SBAND_PROFILE) {
        g_ground_pending_sband_profile = 0u;
    }

    printf("CMD_ACK,cmd=%u,ack_seq=%u,status=%u,"
           "cmd_rssi_x100=%d,cmd_snr_x100=%d,"
           "rssi_x100=%d,snr_x100=%d\n",
           ack->cmd_id, ack->ack_seq, ack->status,
           (int)ack->cmd_rssi_x100, (int)ack->cmd_snr_x100,
           (int)rssi_x100, (int)snr_x100);
}

static void ground_print_bulk(band_t rx_band, const balloon_bulk_t *bulk,
                               int16_t rssi_x100, int16_t snr_x100) {
    char hex_buf[BALLOON_BULK_MAX_PAYLOAD * 2 + 1];
    uint16_t print_len = (bulk->data_len > 32u) ? 32u : bulk->data_len;
    hex_encode(bulk->data, print_len, hex_buf);
    printf("BULK,band=%s,seq=%u,seed=%u,len=%u,"
           "rssi_x100=%d,snr_x100=%d,hex=%s\n",
           band_str(rx_band), bulk->h.seq, bulk->seed, bulk->data_len,
           (int)rssi_x100, (int)snr_x100, hex_buf);
}

static void ground_print_image_packet(const image_packet_t *img,
                                       int16_t rssi_x100, int16_t snr_x100) {
    switch ((image_pkt_type_t)img->pkt_type) {
    case IMAGE_PKT_START:
        printf("IMG_START,total_pkts=%u\n", img->total_pkts);
        break;
    case IMAGE_PKT_DATA: {
        char hex_buf[IMAGE_DATA_PER_PKT * 2 + 1];
        hex_encode(img->data, img->data_len, hex_buf);
        printf("IMG_DATA,pkt=%u,len=%u,rssi_x100=%d,snr_x100=%d,hex=%s\n",
               img->pkt_num, img->data_len,
               (int)rssi_x100, (int)snr_x100, hex_buf);
        break;
    }
    case IMAGE_PKT_END:
        printf("IMG_END,total_pkts=%u\n", img->total_pkts);
        break;
    case IMAGE_PKT_NACK:
        printf("IMG_NACK,len=%u\n", img->data_len);
        break;
    default:
        break;
    }
}

static void ground_handle_rx(band_t rx_band, raw_frame_t *frame) {
    /* UHF frames: strip callsign first */
    if (rx_band == BAND_UHF) {
        uhf_strip_callsign(frame);
    }

    if (frame->length < sizeof(balloon_header_t)) return;

    uint16_t magic;
    memcpy(&magic, frame->data, 2u);

    /* Balloon protocol packet */
    if (magic == BALLOON_MAGIC) {
        const balloon_header_t *hdr = (const balloon_header_t *)frame->data;
        switch ((balloon_msg_type_t)hdr->msg_type) {
        case BALLOON_MSG_BEACON:
            if (frame->length >= sizeof(balloon_beacon_t)) {
                ground_print_beacon(rx_band,
                    (const balloon_beacon_t *)frame->data,
                    frame->rssi_dbm_x100, frame->snr_db_x100);
            }
            break;
        case BALLOON_MSG_CMD_ACK:
            if (frame->length >= sizeof(balloon_cmd_ack_t)) {
                ground_print_cmd_ack(
                    (const balloon_cmd_ack_t *)frame->data,
                    frame->rssi_dbm_x100, frame->snr_db_x100);
            }
            break;
        case BALLOON_MSG_BULK:
            if (frame->length >= BALLOON_BULK_HEADER_SIZE) {
                const balloon_bulk_t *bulk = (const balloon_bulk_t *)frame->data;
                uint16_t bulk_len = bulk->data_len;
                if (bulk_len <= BALLOON_BULK_MAX_PAYLOAD &&
                    frame->length >= (uint16_t)(BALLOON_BULK_HEADER_SIZE + bulk_len)) {
                ground_print_bulk(rx_band,
                    bulk,
                    frame->rssi_dbm_x100, frame->snr_db_x100);
                }
            }
            break;
        default:
            break;
        }
        return;
    }

    /* Image protocol packet (S-Band only) */
    if (magic == IMAGE_PACKET_MAGIC && frame->length >= IMAGE_HEADER_SIZE) {
        ground_print_image_packet(
            (const image_packet_t *)frame->data,
            frame->rssi_dbm_x100, frame->snr_db_x100);
        return;
    }
}

static void ground_send_command(balloon_cmd_id_t cmd_id,
                                 uint8_t param1, uint8_t param2,
                                 int seq_override) {
    balloon_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.h.magic = BALLOON_MAGIC;
    cmd.h.msg_type = BALLOON_MSG_CMD;
    cmd.h.seq = (seq_override >= 0) ? (uint8_t)seq_override : g_ground_cmd_seq++;
    cmd.cmd_id = (uint8_t)cmd_id;
    cmd.param1 = param1;
    cmd.param2 = param2;

    if (cmd_id == BALLOON_CMD_SET_SBAND_PROFILE &&
        param1 >= 1u && param1 <= 4u) {
        g_ground_pending_sband_profile = param1;
    }

    uhf_send_with_callsign(&cmd, sizeof(cmd));
    printf("CMD_SENT,cmd=%u,param1=%u,param2=%u,seq=%u\n",
           cmd.cmd_id, cmd.param1, cmd.param2, cmd.h.seq);
}

static void ground_send_image_nack(const uint16_t *missing,
                                    uint16_t count) {
    image_packet_t nack;
    memset(&nack, 0, sizeof(nack));
    nack.magic = IMAGE_PACKET_MAGIC;
    nack.pkt_type = IMAGE_PKT_NACK;

    /* Pack as many 16-bit pkt numbers as fit within UHF payload limit */
    uint16_t max_nack_bytes = UHF_MAX_PAYLOAD - IMAGE_HEADER_SIZE;
    uint16_t max_nacks = max_nack_bytes / 2u;
    uint16_t send_count = (count > max_nacks) ? max_nacks : count;
    nack.data_len = (uint8_t)(send_count * 2u);
    memcpy(nack.data, missing, send_count * 2u);

    /* Send NACK on UHF so balloon can hear it */
    uhf_send_with_callsign(&nack, IMAGE_HEADER_SIZE + nack.data_len);
    printf("NACK_SENT,count=%u\n", send_count);
}

static void ground_handle_serial_cmd(void) {
    if (strncmp(g_cmd_buf, "CMD,", 4) != 0) return;

    const char *cmd_name = find_field(g_cmd_buf, "cmd=");
    if (!cmd_name) return;

    int seq = parse_int_field(g_cmd_buf, "seq=", -1);
    int param1 = parse_int_field(g_cmd_buf, "param1=", 0);
    int param2 = parse_int_field(g_cmd_buf, "param2=", 0);

    if (strncmp(cmd_name, "PING", 4) == 0) {
        ground_send_command(BALLOON_CMD_PING, 0, 0, seq);
    } else if (strncmp(cmd_name, "SET_SBAND_PROFILE", 17) == 0) {
        ground_send_command(BALLOON_CMD_SET_SBAND_PROFILE,
                            (uint8_t)param1, 0, seq);
    } else if (strncmp(cmd_name, "START_IMAGE", 11) == 0) {
        ground_send_command(BALLOON_CMD_START_IMAGE, (uint8_t)param1, 0, seq);
    } else if (strncmp(cmd_name, "START_BULK", 10) == 0) {
        ground_send_command(BALLOON_CMD_START_BULK,
                            (uint8_t)param1, (uint8_t)param2, seq);
    } else if (strncmp(cmd_name, "STOP", 4) == 0) {
        ground_send_command(BALLOON_CMD_STOP, 0, 0, seq);
    } else if (strncmp(cmd_name, "POWER_SWEEP", 11) == 0) {
        ground_send_command(BALLOON_CMD_POWER_SWEEP,
                            (uint8_t)param1, (uint8_t)param2, seq);
    } else if (strncmp(cmd_name, "IMG_NACK", 8) == 0) {
        /* Parse hex list of 16-bit packet numbers */
        const char *hex = find_field(g_cmd_buf, "hex=");
        if (hex) {
            uint16_t missing[IMAGE_DATA_PER_PKT / 2u];
            uint16_t nbytes = hex_decode(hex, (uint8_t *)missing,
                                          sizeof(missing));
            uint16_t count = nbytes / 2u;
            ground_send_image_nack(missing, count);
        }
    } else {
        printf("ERROR,unknown_cmd\n");
    }
}

static int run_ground(void) {
    printf("[balloon] GROUND role — starting\n");
    printf("READY\n");

    dual_rx_ctx_t rx_ctx;
    dual_rx_init(&rx_ctx);

    while (true) {
        /* Poll USB serial for commands from Python */
        if (try_read_line()) {
            ground_handle_serial_cmd();
        }

        /* Poll both radios */
        band_t rx_band;
        raw_frame_t frame;
        if (dual_rx_poll(&rx_ctx, &rx_band, &frame)) {
            ground_handle_rx(rx_band, &frame);
        }

        sleep_ms(MAIN_LOOP_SLEEP_MS);
    }
}

#endif /* BALLOON_FORCE_ROLE == 2 */

/* ------------------------------------------------------------------ */
/* Init & main                                                        */
/* ------------------------------------------------------------------ */

static int balloon_test_run(void) {
    stdio_init_all();
    sleep_ms(BOOT_DELAY_MS);

    printf("[balloon] Balloon Test — init\n");

    board_pins_init();
    led_init();
    led_off();

    /* Init both radios */
    if (band_init(BAND_SBAND) == RADIO_STATUS_OK) {
        g_sband_ok = true;
        (void)band_set_profile(BAND_SBAND, SBAND_DEFAULT_PROFILE);
        (void)band_set_power(BAND_SBAND, SBAND_DEFAULT_POWER);
        printf("[balloon] S-Band OK (profile %u, %d dBm)\n",
               SBAND_DEFAULT_PROFILE, SBAND_DEFAULT_POWER);
    } else {
        printf("[balloon] S-Band init FAILED\n");
    }

    if (band_init(BAND_UHF) == RADIO_STATUS_OK) {
        g_uhf_ok = true;
        (void)band_set_profile(BAND_UHF, UHF_PROFILE);
        (void)band_set_power(BAND_UHF, UHF_POWER_DBM);
        printf("[balloon] UHF OK (profile %u, %d dBm)\n",
               UHF_PROFILE, UHF_POWER_DBM);
    } else {
        printf("[balloon] UHF init FAILED\n");
    }

    if (!g_sband_ok && !g_uhf_ok) {
        printf("[balloon] FATAL: no radios available\n");
        while (true) { led_orange_blink(); sleep_ms(500); }
    }

#if (BALLOON_FORCE_ROLE == 1)
    printf("[balloon] Role: FLIGHT\n");
    return run_flight();
#elif (BALLOON_FORCE_ROLE == 2)
    printf("[balloon] Role: GROUND\n");
    return run_ground();
#else
    printf("[balloon] ERROR: BALLOON_FORCE_ROLE not set (compile with 1 or 2)\n");
    while (true) { led_orange_blink(); sleep_ms(500); }
#endif
}

int main(void) {
    return balloon_test_run();
}
