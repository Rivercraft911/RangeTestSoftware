/*
 * balloon_link_test.c — Dual-role balloon link-budget test firmware
 *
 * Compile with BALLOON_FORCE_ROLE=1 for flight, BALLOON_FORCE_ROLE=2 for ground.
 * Flight: beacons on UHF, responds to commands, downlinks images/bulk over S-Band.
 * Ground: dual-band RX, prints KEY,field=value lines, forwards host commands over UHF.
 */

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

/* ---- Configuration ---- */
#define BOOT_DELAY_MS       2000u
#define BEACON_INTERVAL_MS  2500u
#define TX_TIMEOUT_MS       2000u
#define UHF_RX_TIMEOUT_MS   3000u
#define SBAND_RX_TIMEOUT_MS 5000u
#define BULK_INTER_PKT_MS   15u
#define ARQ_RX_TIMEOUT_MS   1000u
#define ARQ_MAX_ROUNDS      3u
#define SWEEP_PKTS_PER_LEVEL 5u
#define SWEEP_STEP_DBM      2
#define CMD_BUF_SIZE        600u

#define DEFAULT_SBAND_PROFILE       1u
#define DEFAULT_SBAND_TX_POWER_DBM  13
#define DEFAULT_UHF_TX_POWER_DBM    20

#define UHF_BULK_DATA_BYTES         64u
#define UHF_BULK_CTRL_LISTEN_MS     120u

#define CMD_RESULT_INVALID_PARAM    1u
#define CMD_RESULT_UNAVAILABLE      2u
#define CMD_RESULT_BUSY             3u
#define CMD_RESULT_RADIO_ERR        4u

/* ---- Role selection ---- */
#ifndef BALLOON_FORCE_ROLE
#error "Define BALLOON_FORCE_ROLE=1 (flight) or BALLOON_FORCE_ROLE=2 (ground)"
#endif
#define IS_FLIGHT  (BALLOON_FORCE_ROLE == 1)
#define IS_GROUND  (BALLOON_FORCE_ROLE == 2)

/* ---- Serial line buffer ---- */
static char g_cmd_buf[CMD_BUF_SIZE];
static uint16_t g_cmd_pos = 0u;

/* ---- LED (WS2812 NeoPixel) ---- */
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

static void led_off(void)    { led_set(0, 0, 0); }
static void led_red(void)    { led_set(40, 0, 0); }
static void led_green(void)  { led_set(0, 40, 0); }
static void led_blue(void)   { led_set(0, 0, 40); }
static void led_yellow(void) { led_set(40, 30, 0); }
static void led_purple(void) { led_set(30, 0, 40); }

/* ---- Serial helpers ---- */

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
        if (g_cmd_pos < CMD_BUF_SIZE - 1u)
            g_cmd_buf[g_cmd_pos++] = (char)ch;
    }
}

static const char *find_field(const char *line, const char *key) {
    const char *p = strstr(line, key);
    return p ? p + strlen(key) : NULL;
}

static int parse_int_field(const char *line, const char *key, int fallback) {
    const char *v = find_field(line, key);
    return v ? atoi(v) : fallback;
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

/* ---- Radio wrappers ---- */

static bool uhf_wait_tx_done(void) {
    uint64_t deadline = to_ms_since_boot(get_absolute_time()) + TX_TIMEOUT_MS;
    while (to_ms_since_boot(get_absolute_time()) < deadline) {
        radio_event_t ev = RADIO_EVENT_NONE;
        rfm98pw_poll_event(&ev);
        if (ev == RADIO_EVENT_TX_DONE) return true;
        if (ev == RADIO_EVENT_TIMEOUT || ev == RADIO_EVENT_ERROR) {
            rfm98pw_abort();
            return false;
        }
        sleep_us(100);
    }
    rfm98pw_abort();
    return false;
}

static bool sband_wait_tx_done(void) {
    uint64_t deadline = to_ms_since_boot(get_absolute_time()) + TX_TIMEOUT_MS;
    while (to_ms_since_boot(get_absolute_time()) < deadline) {
        radio_event_t ev = RADIO_EVENT_NONE;
        sx1280f27_poll_event(&ev);
        if (ev == RADIO_EVENT_TX_DONE) return true;
        if (ev == RADIO_EVENT_TIMEOUT || ev == RADIO_EVENT_ERROR) {
            sx1280f27_abort();
            return false;
        }
        sleep_us(100);
    }
    sx1280f27_abort();
    return false;
}

static bool uhf_send(const uint8_t *data, uint8_t len) {
    uint8_t buf[255];
    memcpy(buf, data, len);
    uint8_t total = uhf_append_callsign(buf, len);
    rfm98pw_abort();  /* must be IDLE before TX */
    radio_status_t st = rfm98pw_start_tx(buf, total, TX_TIMEOUT_MS);
    if (st != RADIO_STATUS_OK) return false;
    return uhf_wait_tx_done();
}

static bool sband_send(const uint8_t *data, uint8_t len) {
    sx1280f27_abort();  /* must be IDLE before TX */
    radio_status_t st = sx1280f27_start_tx(data, len, TX_TIMEOUT_MS);
    if (st != RADIO_STATUS_OK) return false;
    return sband_wait_tx_done();
}

/* ---- PRBS-15 generator ---- */
static uint16_t g_prbs_state = 0x7FFFu;

static uint8_t prbs15_next(void) {
    uint8_t out = 0u;
    for (int i = 0; i < 8; i++) {
        uint16_t bit = ((g_prbs_state >> 14) ^ (g_prbs_state >> 13)) & 1u;
        g_prbs_state = (uint16_t)((g_prbs_state << 1) | bit) & 0x7FFFu;
        out = (uint8_t)((out << 1) | bit);
    }
    return out;
}

/* ====================================================================
 * FLIGHT ROLE
 * ==================================================================== */
#if IS_FLIGHT

#include "data/balloon_text.h"

/* Image data — define BALLOON_HAS_IMAGE in CMake when balloon_image.h exists */
#ifdef BALLOON_HAS_IMAGE
#include "data/balloon_image.h"
#define IMAGE_DATA      balloon_image
#define IMAGE_DATA_LEN  BALLOON_IMAGE_SIZE
#else
static const uint8_t IMAGE_DATA[] = {0};
static const uint32_t IMAGE_DATA_LEN = 0;
#endif

static balloon_state_t g_state = BALLOON_STATE_BEACON;
static uint8_t g_seq = 0u;
static uint8_t g_sband_profile = DEFAULT_SBAND_PROFILE;
static int8_t  g_tx_power_dbm = DEFAULT_UHF_TX_POWER_DBM;
static uint32_t g_tx_count = 0u;
static uint32_t g_rx_count = 0u;
static int16_t g_last_cmd_rssi = 0;
static int16_t g_last_cmd_snr = 0;

/* Sweep state */
static int8_t  g_sweep_min_dbm = 2;
static int8_t  g_sweep_max_dbm = 13;
static int8_t  g_sweep_cur_dbm = 2;
static uint8_t g_sweep_pkt_count = 0u;
static uint8_t g_sweep_band = BALLOON_BAND_SBAND;

/* Bulk state */
static uint8_t  g_bulk_band = BALLOON_BAND_SBAND;
static uint32_t g_bulk_offset = 0u;
static uint16_t g_bulk_pkt_num = 0u;

static void flight_send_beacon(void) {
    balloon_beacon_t bcn;
    memset(&bcn, 0, sizeof(bcn));
    bcn.hdr.magic = BALLOON_MAGIC;
    bcn.hdr.msg_type = BALLOON_MSG_BEACON;
    bcn.hdr.seq = g_seq++;
    bcn.uptime_ms = to_ms_since_boot(get_absolute_time());
    bcn.tx_count = g_tx_count;
    bcn.rx_count = g_rx_count;
    bcn.last_cmd_rssi_x100 = g_last_cmd_rssi;
    bcn.last_cmd_snr_x100 = g_last_cmd_snr;
    bcn.tx_power_dbm = g_tx_power_dbm;
    bcn.sband_profile = g_sband_profile;
    bcn.state = (uint8_t)g_state;
    bcn.flags = BALLOON_FLAG_UHF_OK | BALLOON_FLAG_SBAND_OK;
#ifdef BALLOON_HAS_IMAGE
    bcn.flags |= BALLOON_FLAG_IMAGE_LOADED;
#endif

    led_yellow();
    if (uhf_send((const uint8_t *)&bcn, sizeof(bcn)))
        g_tx_count++;
    led_off();
}

static void flight_send_ack(uint8_t cmd_seq, uint8_t cmd_id, uint8_t result,
                             int16_t rssi, int16_t snr) {
    balloon_cmd_ack_t ack;
    memset(&ack, 0, sizeof(ack));
    ack.hdr.magic = BALLOON_MAGIC;
    ack.hdr.msg_type = BALLOON_MSG_CMD_ACK;
    ack.hdr.seq = cmd_seq;
    ack.cmd_id = cmd_id;
    ack.result = result;
    ack.cmd_rssi_x100 = rssi;
    ack.cmd_snr_x100 = snr;

    if (uhf_send((const uint8_t *)&ack, sizeof(ack)))
        g_tx_count++;
}

static void flight_handle_cmd(const balloon_cmd_t *cmd,
                               int16_t rssi, int16_t snr) {
    g_rx_count++;
    g_last_cmd_rssi = rssi;
    g_last_cmd_snr = snr;

    uint8_t result = 0u;

    switch (cmd->cmd_id) {
    case BALLOON_CMD_PING:
        break;

    case BALLOON_CMD_SET_SBAND_PROF:
        if (cmd->param >= 1u && cmd->param <= 4u) {
            sx1280f27_abort();
            if (sx1280f27_set_profile(cmd->param) == RADIO_STATUS_OK) {
                g_sband_profile = cmd->param;
            } else {
                result = CMD_RESULT_RADIO_ERR;
            }
        } else {
            result = CMD_RESULT_INVALID_PARAM;
        }
        break;

    case BALLOON_CMD_START_IMAGE:
#ifdef BALLOON_HAS_IMAGE
        g_state = BALLOON_STATE_IMAGE;
#else
        result = CMD_RESULT_UNAVAILABLE;
#endif
        break;

    case BALLOON_CMD_START_BULK:
        g_bulk_band = (cmd->param == BALLOON_BAND_UHF) ?
                      BALLOON_BAND_UHF : BALLOON_BAND_SBAND;
        g_bulk_offset = 0u;
        g_bulk_pkt_num = 0u;
        g_prbs_state = 0x7FFFu;
        g_state = BALLOON_STATE_BULK;
        break;

    case BALLOON_CMD_STOP:
        g_state = BALLOON_STATE_BEACON;
        break;

    case BALLOON_CMD_POWER_SWEEP:
        g_sweep_min_dbm = 2;
        if (cmd->param >= 32u) {
            /* Encoded: band in bits [6:5], max_power in bits [4:0] */
            uint8_t enc_band = (cmd->param >> 5) & 0x03;
            g_sweep_band = (enc_band == BALLOON_BAND_UHF) ?
                           BALLOON_BAND_UHF : BALLOON_BAND_SBAND;
            g_sweep_max_dbm = (int8_t)(cmd->param & 0x1F);
        } else {
            g_sweep_band = BALLOON_BAND_SBAND;
            g_sweep_max_dbm = (cmd->param > 0) ? (int8_t)cmd->param : 13;
        }
        if (g_sweep_max_dbm < 2) g_sweep_max_dbm = 13;
        g_sweep_cur_dbm = g_sweep_min_dbm;
        g_sweep_pkt_count = 0u;
        if (g_sweep_band == BALLOON_BAND_SBAND)
            sx1280f27_set_tx_power_dbm(g_sweep_cur_dbm);
        else
            rfm98pw_set_tx_power_dbm(g_sweep_cur_dbm);
        g_state = BALLOON_STATE_POWER_SWEEP;
        break;

    default:
        result = 0xFFu;
        break;
    }

    flight_send_ack(cmd->hdr.seq, cmd->cmd_id, result, rssi, snr);
}

static bool flight_cmd_allowed_while_busy(uint8_t cmd_id) {
    return cmd_id == BALLOON_CMD_PING || cmd_id == BALLOON_CMD_STOP;
}

static bool flight_poll_uhf_cmd_window(uint32_t listen_ms, bool busy_only,
                                       bool *saw_stop) {
    if (saw_stop) *saw_stop = false;
    rfm98pw_start_rx(listen_ms);
    uint64_t deadline = to_ms_since_boot(get_absolute_time()) + listen_ms;
    while (to_ms_since_boot(get_absolute_time()) < deadline) {
        radio_event_t ev = RADIO_EVENT_NONE;
        rfm98pw_poll_event(&ev);
        if (ev == RADIO_EVENT_RX_DONE) {
            radio_rx_frame_t frame;
            if (rfm98pw_read_rx(&frame) == RADIO_STATUS_OK &&
                frame.length >= sizeof(balloon_cmd_t)) {
                balloon_cmd_t cmd;
                memcpy(&cmd, frame.data, sizeof(cmd));
                if (cmd.hdr.magic == BALLOON_MAGIC &&
                    cmd.hdr.msg_type == BALLOON_MSG_CMD) {
                    if (busy_only && !flight_cmd_allowed_while_busy(cmd.cmd_id)) {
                        g_rx_count++;
                        g_last_cmd_rssi = frame.rssi_dbm_x100;
                        g_last_cmd_snr = frame.snr_db_x100;
                        flight_send_ack(cmd.hdr.seq, cmd.cmd_id, CMD_RESULT_BUSY,
                                        frame.rssi_dbm_x100,
                                        frame.snr_db_x100);
                    } else {
                        led_green();  /* flash green = command received */
                        flight_handle_cmd(&cmd, frame.rssi_dbm_x100,
                                          frame.snr_db_x100);
                    }
                    if (saw_stop && cmd.cmd_id == BALLOON_CMD_STOP)
                        *saw_stop = true;
                    return true;
                }
            }
            rfm98pw_start_rx(listen_ms);
        } else if (ev == RADIO_EVENT_TIMEOUT) {
            return false;
        } else if (ev == RADIO_EVENT_CRC_FAIL || ev == RADIO_EVENT_ERROR) {
            rfm98pw_abort();
            rfm98pw_start_rx(listen_ms);
        }
        sleep_us(200);
    }
    rfm98pw_abort();
    return false;
}

static bool flight_poll_uhf_cmd(uint32_t listen_ms) {
    return flight_poll_uhf_cmd_window(listen_ms, false, NULL);
}

/* Quick UHF control poll during long operations: allow PING/STOP, reject others. */
static bool flight_check_uhf_busy_cmd(uint32_t listen_ms) {
    bool saw_stop = false;
    (void)flight_poll_uhf_cmd_window(listen_ms, true, &saw_stop);
    return saw_stop;
}

/* ---- Image downlink ---- */

static void flight_run_image(void) {
#ifndef BALLOON_HAS_IMAGE
    g_state = BALLOON_STATE_BEACON;
    return;
#endif

    uint32_t data_per_pkt = BALLOON_BULK_DATA_MAX;
    uint16_t total = (uint16_t)((IMAGE_DATA_LEN + data_per_pkt - 1u) / data_per_pkt);

    led_purple();

    for (uint16_t i = 0; i < total; i++) {
        balloon_bulk_t pkt;
        memset(&pkt, 0, sizeof(balloon_bulk_t));
        pkt.hdr.magic = BALLOON_MAGIC;
        pkt.hdr.msg_type = BALLOON_MSG_BULK;
        pkt.hdr.seq = g_seq++;
        pkt.pkt_num = i;
        pkt.total_pkts = total;
        pkt.band = BALLOON_BAND_SBAND;

        uint32_t offset = (uint32_t)i * data_per_pkt;
        uint32_t remaining = IMAGE_DATA_LEN - offset;
        uint8_t chunk = (remaining > data_per_pkt) ?
                        (uint8_t)data_per_pkt : (uint8_t)remaining;
        pkt.data_len = chunk;
        memcpy(pkt.data, &IMAGE_DATA[offset], chunk);

        uint8_t tx_len = BALLOON_BULK_HDR_SIZE + chunk;
        if (sband_send((const uint8_t *)&pkt, tx_len))
            g_tx_count++;

        if ((i & 0x0F) == 0) led_purple();
        else if ((i & 0x0F) == 8) led_off();

        sleep_ms(BULK_INTER_PKT_MS);

        if ((i & 0x07u) == 0x07u) {
            if (flight_check_uhf_busy_cmd(40u)) return;
        }

        if (g_state != BALLOON_STATE_IMAGE) return;
    }

    /* ARQ: listen for NACK lists from ground */
    for (uint8_t round = 0; round < ARQ_MAX_ROUNDS; round++) {
        sx1280f27_start_rx(ARQ_RX_TIMEOUT_MS);
        bool got_nack = false;
        uint64_t deadline = to_ms_since_boot(get_absolute_time()) + ARQ_RX_TIMEOUT_MS;
        uint64_t next_cmd_poll = to_ms_since_boot(get_absolute_time());

        while (to_ms_since_boot(get_absolute_time()) < deadline) {
            uint64_t now_ms = to_ms_since_boot(get_absolute_time());
            if (now_ms >= next_cmd_poll) {
                if (flight_check_uhf_busy_cmd(20u)) {
                    sx1280f27_abort();
                    led_off();
                    return;
                }
                next_cmd_poll = now_ms + 75u;
            }

            radio_event_t ev = RADIO_EVENT_NONE;
            sx1280f27_poll_event(&ev);
            if (ev == RADIO_EVENT_RX_DONE) {
                radio_rx_frame_t frame;
                if (sx1280f27_read_rx(&frame) == RADIO_STATUS_OK &&
                    frame.length >= IMAGE_HEADER_SIZE) {
                    image_packet_t nack;
                    memcpy(&nack, frame.data, IMAGE_HEADER_SIZE);
                    if (nack.magic == IMAGE_PACKET_MAGIC &&
                        nack.pkt_type == IMAGE_PKT_NACK) {
                        /* Retransmit requested packets */
                        uint8_t nack_bytes = frame.length - IMAGE_HEADER_SIZE;
                        if (nack_bytes > IMAGE_DATA_PER_PKT) nack_bytes = IMAGE_DATA_PER_PKT;
                        memcpy(nack.data, &frame.data[IMAGE_HEADER_SIZE], nack_bytes);
                        uint16_t missing = nack_bytes / 2u;
                        uint16_t *list = (uint16_t *)nack.data;

                        for (uint16_t m = 0; m < missing; m++) {
                            uint16_t idx = list[m];
                            if (idx >= total) continue;

                            balloon_bulk_t rpkt;
                            memset(&rpkt, 0, sizeof(balloon_bulk_t));
                            rpkt.hdr.magic = BALLOON_MAGIC;
                            rpkt.hdr.msg_type = BALLOON_MSG_BULK;
                            rpkt.hdr.seq = g_seq++;
                            rpkt.pkt_num = idx;
                            rpkt.total_pkts = total;
                            rpkt.band = BALLOON_BAND_SBAND;

                            uint32_t off = (uint32_t)idx * data_per_pkt;
                            uint32_t rem = IMAGE_DATA_LEN - off;
                            uint8_t ch = (rem > data_per_pkt) ?
                                         (uint8_t)data_per_pkt : (uint8_t)rem;
                            rpkt.data_len = ch;
                            memcpy(rpkt.data, &IMAGE_DATA[off], ch);

                            uint8_t tl = BALLOON_BULK_HDR_SIZE + ch;
                            if (sband_send((const uint8_t *)&rpkt, tl))
                                g_tx_count++;
                            sleep_ms(BULK_INTER_PKT_MS);
                        }
                        got_nack = true;
                    }
                }
                break;
            }
            if (ev == RADIO_EVENT_TIMEOUT) break;
            if (ev == RADIO_EVENT_CRC_FAIL || ev == RADIO_EVENT_ERROR) {
                sx1280f27_abort();
                sx1280f27_start_rx(ARQ_RX_TIMEOUT_MS);
            }
            sleep_us(200);
        }
        sx1280f27_abort();
        if (!got_nack) break;
    }

    led_green();
    sleep_ms(500);
    led_off();
    g_state = BALLOON_STATE_BEACON;
}

/* ---- Bulk stress test ---- */

static void flight_run_bulk(void) {
    led_blue();

    while (g_state == BALLOON_STATE_BULK) {
        balloon_bulk_t pkt;
        memset(&pkt, 0, sizeof(balloon_bulk_t));
        pkt.hdr.magic = BALLOON_MAGIC;
        pkt.hdr.msg_type = BALLOON_MSG_BULK;
        pkt.hdr.seq = g_seq++;
        pkt.pkt_num = g_bulk_pkt_num++;
        pkt.total_pkts = 0u; /* continuous — 0 means streaming */
        pkt.band = g_bulk_band;

        uint8_t data_bytes = (g_bulk_band == BALLOON_BAND_UHF) ?
                             UHF_BULK_DATA_BYTES : BALLOON_BULK_DATA_MAX;

        /* Fill with text data, wrapping around the buffer */
        for (uint8_t i = 0; i < data_bytes; i++) {
            pkt.data[i] = (uint8_t)BALLOON_BULK_TEXT[g_bulk_offset % BALLOON_BULK_TEXT_LEN];
            g_bulk_offset++;
        }
        pkt.data_len = data_bytes;

        uint8_t tx_len = (uint8_t)(BALLOON_BULK_HDR_SIZE + data_bytes);
        bool ok;
        if (g_bulk_band == BALLOON_BAND_SBAND)
            ok = sband_send((const uint8_t *)&pkt, tx_len);
        else
            ok = uhf_send((const uint8_t *)&pkt, tx_len);

        if (ok) g_tx_count++;

        if ((g_bulk_pkt_num & 0x07) == 0) led_blue();
        else if ((g_bulk_pkt_num & 0x07) == 4) led_off();

        if (g_bulk_band == BALLOON_BAND_UHF) {
            if (flight_check_uhf_busy_cmd(UHF_BULK_CTRL_LISTEN_MS)) return;
        }

        sleep_ms(BULK_INTER_PKT_MS);

        /* Check for UHF control periodically during S-Band bulk */
        if (g_bulk_band == BALLOON_BAND_SBAND &&
            (g_bulk_pkt_num & 0x03) == 0) {
            if (flight_check_uhf_busy_cmd(200u)) return;
        }
    }
    led_off();
}

/* ---- Power sweep ---- */

static void flight_run_power_sweep(void) {
    led_green();

    while (g_state == BALLOON_STATE_POWER_SWEEP) {
        if (g_sweep_band == BALLOON_BAND_SBAND) {
            sx1280f27_set_tx_power_dbm(g_sweep_cur_dbm);
        } else {
            rfm98pw_set_tx_power_dbm(g_sweep_cur_dbm);
            g_tx_power_dbm = g_sweep_cur_dbm;
        }

        for (uint8_t i = 0; i < SWEEP_PKTS_PER_LEVEL; i++) {
            /* Beacon on UHF (also the test pkt for UHF sweep) */
            flight_send_beacon();

            /* For S-Band sweep, send an S-Band test packet too */
            if (g_sweep_band == BALLOON_BAND_SBAND) {
                balloon_bulk_t pkt;
                memset(&pkt, 0, sizeof(balloon_bulk_t));
                pkt.hdr.magic = BALLOON_MAGIC;
                pkt.hdr.msg_type = BALLOON_MSG_BULK;
                pkt.hdr.seq = g_seq++;
                pkt.pkt_num = g_sweep_pkt_count++;
                pkt.total_pkts = 0u;
                pkt.band = BALLOON_BAND_SBAND;
                pkt.data_len = 10u;
                if (sband_send((const uint8_t *)&pkt, BALLOON_BULK_HDR_SIZE + 10u))
                    g_tx_count++;
            }
            sleep_ms(500);

            if (flight_check_uhf_busy_cmd(40u)) return;
        }

        g_sweep_cur_dbm += SWEEP_STEP_DBM;
        if (g_sweep_cur_dbm > g_sweep_max_dbm) {
            /* Reset to default and return to beacon */
            if (g_sweep_band == BALLOON_BAND_SBAND) {
                sx1280f27_set_tx_power_dbm(DEFAULT_SBAND_TX_POWER_DBM);
                g_tx_power_dbm = DEFAULT_UHF_TX_POWER_DBM;
            } else {
                rfm98pw_set_tx_power_dbm(DEFAULT_UHF_TX_POWER_DBM);
                g_tx_power_dbm = DEFAULT_UHF_TX_POWER_DBM;
            }
            g_state = BALLOON_STATE_BEACON;
        }

        if (flight_check_uhf_busy_cmd(40u)) return;
    }
    led_off();
}

/* ---- Flight main ---- */

static void flight_main(void) {
    printf("BalloonFlight ready\n");

    while (true) {
        switch (g_state) {
        case BALLOON_STATE_BEACON:
            flight_send_beacon();
            led_blue();  /* dim blue = listening for commands */
            flight_poll_uhf_cmd(BEACON_INTERVAL_MS - 200u);
            led_off();
            break;

        case BALLOON_STATE_IMAGE:
            flight_run_image();
            break;

        case BALLOON_STATE_BULK:
            flight_run_bulk();
            break;

        case BALLOON_STATE_POWER_SWEEP:
            flight_run_power_sweep();
            break;

        default:
            g_state = BALLOON_STATE_BEACON;
            break;
        }
    }
}

#endif /* IS_FLIGHT */

/* ====================================================================
 * GROUND ROLE
 * ==================================================================== */
#if IS_GROUND

static uint8_t g_ground_seq = 0u;

static void ground_print_beacon(const balloon_beacon_t *bcn,
                                 int16_t rssi, int16_t snr) {
    printf("BEACON,uptime=%lu,state=%u,sband_prof=%u,tx_pwr=%d,"
           "flags=%u,tx_cnt=%lu,rx_cnt=%lu,"
           "cmd_rssi=%d,cmd_snr=%d,rssi=%d,snr=%d\n",
           (unsigned long)bcn->uptime_ms,
           (unsigned)bcn->state,
           (unsigned)bcn->sband_profile,
           (int)bcn->tx_power_dbm,
           (unsigned)bcn->flags,
           (unsigned long)bcn->tx_count,
           (unsigned long)bcn->rx_count,
           (int)bcn->last_cmd_rssi_x100,
           (int)bcn->last_cmd_snr_x100,
           (int)rssi,
           (int)snr);
}

static void ground_print_ack(const balloon_cmd_ack_t *ack,
                              int16_t rssi, int16_t snr) {
    printf("CMD_ACK,cmd=%u,seq=%u,result=%u,"
           "cmd_rssi=%d,cmd_snr=%d,rssi=%d,snr=%d\n",
           (unsigned)ack->cmd_id,
           (unsigned)ack->hdr.seq,
           (unsigned)ack->result,
           (int)ack->cmd_rssi_x100,
           (int)ack->cmd_snr_x100,
           (int)rssi,
           (int)snr);
}

static const char HEX_LUT[] = "0123456789ABCDEF";

static void ground_print_bulk(const balloon_bulk_t *pkt, uint8_t data_len,
                               int16_t rssi, int16_t snr) {
    static char hex_buf[BALLOON_BULK_DATA_MAX * 2 + 1];
    for (uint8_t i = 0; i < data_len; i++) {
        hex_buf[i * 2]     = HEX_LUT[pkt->data[i] >> 4];
        hex_buf[i * 2 + 1] = HEX_LUT[pkt->data[i] & 0x0F];
    }
    hex_buf[data_len * 2] = '\0';
    printf("BULK,pkt=%u,total=%u,len=%u,band=%u,"
           "rssi=%d,snr=%d,hex=%s\n",
           (unsigned)pkt->pkt_num,
           (unsigned)pkt->total_pkts,
           (unsigned)data_len,
           (unsigned)pkt->band,
           (int)rssi,
           (int)snr,
           hex_buf);
}

static void ground_handle_rx(const radio_rx_frame_t *frame, uint8_t band) {
    if (frame->length < sizeof(balloon_header_t)) return;

    balloon_header_t hdr;
    memcpy(&hdr, frame->data, sizeof(hdr));
    if (hdr.magic != BALLOON_MAGIC) return;

    switch (hdr.msg_type) {
    case BALLOON_MSG_BEACON:
        if (frame->length >= sizeof(balloon_beacon_t)) {
            balloon_beacon_t bcn;
            memcpy(&bcn, frame->data, sizeof(bcn));
            ground_print_beacon(&bcn, frame->rssi_dbm_x100,
                                frame->snr_db_x100);
        }
        break;

    case BALLOON_MSG_CMD_ACK:
        if (frame->length >= sizeof(balloon_cmd_ack_t)) {
            balloon_cmd_ack_t ack;
            memcpy(&ack, frame->data, sizeof(ack));
            ground_print_ack(&ack, frame->rssi_dbm_x100,
                             frame->snr_db_x100);
        }
        break;

    case BALLOON_MSG_BULK:
        if (frame->length >= BALLOON_BULK_HDR_SIZE) {
            balloon_bulk_t pkt;
            memset(&pkt, 0, sizeof(pkt));
            uint8_t copy_len = (frame->length < sizeof(pkt)) ?
                               frame->length : (uint8_t)sizeof(pkt);
            memcpy(&pkt, frame->data, copy_len);
            uint8_t data_bytes = frame->length - BALLOON_BULK_HDR_SIZE;
            if (data_bytes > pkt.data_len) data_bytes = pkt.data_len;
            ground_print_bulk(&pkt, data_bytes,
                              frame->rssi_dbm_x100, frame->snr_db_x100);
        }
        break;

    default:
        break;
    }
}

static bool ground_apply_sband_profile(uint8_t profile, uint8_t seq) {
    if (profile < 1u || profile > 4u) {
        printf("CMD_FAIL,cmd=%u,seq=%u,reason=invalid_param\n",
               (unsigned)BALLOON_CMD_SET_SBAND_PROF, (unsigned)seq);
        return false;
    }

    sx1280f27_abort();
    if (sx1280f27_set_profile(profile) != RADIO_STATUS_OK) {
        printf("CMD_FAIL,cmd=%u,seq=%u,reason=local_sband_profile\n",
               (unsigned)BALLOON_CMD_SET_SBAND_PROF, (unsigned)seq);
        return false;
    }

    return true;
}

static void ground_send_cmd(uint8_t cmd_id, uint8_t param, uint8_t seq) {
    balloon_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.hdr.magic = BALLOON_MAGIC;
    cmd.hdr.msg_type = BALLOON_MSG_CMD;
    cmd.hdr.seq = seq;
    cmd.cmd_id = cmd_id;
    cmd.param = param;

    if (uhf_send((const uint8_t *)&cmd, sizeof(cmd))) {
        rfm98pw_start_rx(UHF_RX_TIMEOUT_MS);
        printf("CMD_SENT,cmd=%u,param=%u,seq=%u\n",
               (unsigned)cmd_id, (unsigned)param,
               (unsigned)cmd.hdr.seq);
    } else {
        rfm98pw_start_rx(UHF_RX_TIMEOUT_MS);
        printf("CMD_FAIL,cmd=%u,seq=%u,reason=tx_error\n",
               (unsigned)cmd_id, (unsigned)cmd.hdr.seq);
    }
}

static void ground_send_nack(void) {
    /* Parse: NACK,hex=0005000A000F */
    const char *hex_str = find_field(g_cmd_buf, "hex=");
    if (!hex_str) {
        printf("NACK_ERR,reason=no_hex\n");
        return;
    }

    image_packet_t nack;
    memset(&nack, 0, sizeof(nack));
    nack.magic = IMAGE_PACKET_MAGIC;
    nack.pkt_type = IMAGE_PKT_NACK;

    uint16_t decoded = hex_decode(hex_str, nack.data, IMAGE_DATA_PER_PKT);
    nack.data_len = (uint8_t)decoded;
    nack.pkt_num = decoded / 2u;

    uint8_t tx_len = IMAGE_HEADER_SIZE + (uint8_t)decoded;
    sx1280f27_abort();  /* must be IDLE before TX */
    radio_status_t st = sx1280f27_start_tx((const uint8_t *)&nack, tx_len, TX_TIMEOUT_MS);
    if (st != RADIO_STATUS_OK) {
        printf("NACK_FAIL,reason=tx_start\n");
        return;
    }
    if (sband_wait_tx_done()) {
        printf("NACK_SENT,count=%u\n", (unsigned)(decoded / 2u));
    } else {
        printf("NACK_FAIL,reason=tx_timeout\n");
    }
}

static void ground_process_serial(void) {
    if (strncmp(g_cmd_buf, "CMD,", 4) != 0 &&
        strncmp(g_cmd_buf, "NACK,", 5) != 0)
        return;

    if (strncmp(g_cmd_buf, "NACK,", 5) == 0) {
        ground_send_nack();
        return;
    }

    /* Parse: CMD,cmd=NAME,param=X,seq=Y */
    const char *cmd_name = find_field(g_cmd_buf, "cmd=");
    if (!cmd_name) return;

    uint8_t param = (uint8_t)parse_int_field(g_cmd_buf, "param=", 0);
    const char *seq_field = find_field(g_cmd_buf, "seq=");
    uint8_t seq = seq_field ? (uint8_t)atoi(seq_field) : g_ground_seq++;
    if (seq_field)
        g_ground_seq = (uint8_t)(seq + 1u);

    if (strncmp(cmd_name, "PING", 4) == 0)
        ground_send_cmd(BALLOON_CMD_PING, 0, seq);
    else if (strncmp(cmd_name, "SET_SBAND_PROFILE", 17) == 0) {
        if (ground_apply_sband_profile(param, seq))
            ground_send_cmd(BALLOON_CMD_SET_SBAND_PROF, param, seq);
    } else if (strncmp(cmd_name, "START_IMAGE", 11) == 0)
        ground_send_cmd(BALLOON_CMD_START_IMAGE, 0, seq);
    else if (strncmp(cmd_name, "START_BULK", 10) == 0)
        ground_send_cmd(BALLOON_CMD_START_BULK, param, seq);
    else if (strncmp(cmd_name, "STOP", 4) == 0)
        ground_send_cmd(BALLOON_CMD_STOP, 0, seq);
    else if (strncmp(cmd_name, "POWER_SWEEP", 11) == 0)
        ground_send_cmd(BALLOON_CMD_POWER_SWEEP, param, seq);
    else
        printf("CMD_ERR,reason=unknown_cmd\n");
}

/* ---- Ground main ---- */

static void ground_main(void) {
    printf("BalloonGround ready\n");

    /* Arm both radios for continuous RX */
    rfm98pw_start_rx(UHF_RX_TIMEOUT_MS);
    sx1280f27_start_rx(SBAND_RX_TIMEOUT_MS);
    led_blue();  /* dim blue = listening */

    while (true) {
        /* Check USB serial for commands from host */
        if (try_read_line()) {
            ground_process_serial();
            /* Re-arm radios after TX (uhf_send/sband_send leave radio idle) */
            rfm98pw_start_rx(UHF_RX_TIMEOUT_MS);
            sx1280f27_start_rx(SBAND_RX_TIMEOUT_MS);
        }

        /* Poll UHF */
        radio_event_t uhf_ev = RADIO_EVENT_NONE;
        rfm98pw_poll_event(&uhf_ev);
        if (uhf_ev == RADIO_EVENT_RX_DONE) {
            radio_rx_frame_t frame;
            if (rfm98pw_read_rx(&frame) == RADIO_STATUS_OK) {
                led_green();
                ground_handle_rx(&frame, BALLOON_BAND_UHF);
                led_blue();
            }
            rfm98pw_start_rx(UHF_RX_TIMEOUT_MS);
        } else if (uhf_ev == RADIO_EVENT_TIMEOUT) {
            rfm98pw_start_rx(UHF_RX_TIMEOUT_MS);
        } else if (uhf_ev == RADIO_EVENT_CRC_FAIL || uhf_ev == RADIO_EVENT_ERROR) {
            rfm98pw_abort();
            rfm98pw_start_rx(UHF_RX_TIMEOUT_MS);
        }

        /* Poll S-Band */
        radio_event_t sb_ev = RADIO_EVENT_NONE;
        sx1280f27_poll_event(&sb_ev);
        if (sb_ev == RADIO_EVENT_RX_DONE) {
            radio_rx_frame_t frame;
            if (sx1280f27_read_rx(&frame) == RADIO_STATUS_OK) {
                led_purple();
                ground_handle_rx(&frame, BALLOON_BAND_SBAND);
                led_blue();
            }
            sx1280f27_start_rx(SBAND_RX_TIMEOUT_MS);
        } else if (sb_ev == RADIO_EVENT_TIMEOUT) {
            sx1280f27_start_rx(SBAND_RX_TIMEOUT_MS);
        } else if (sb_ev == RADIO_EVENT_CRC_FAIL || sb_ev == RADIO_EVENT_ERROR) {
            sx1280f27_abort();
            sx1280f27_start_rx(SBAND_RX_TIMEOUT_MS);
        }

        sleep_us(200);
    }
}

#endif /* IS_GROUND */

/* ====================================================================
 * MAIN
 * ==================================================================== */

int main(void) {
    stdio_init_all();
    sleep_ms(BOOT_DELAY_MS);

    board_pins_init();
    led_init();

    /* Init both radios */
    radio_status_t uhf_st = rfm98pw_init();
    if (uhf_st != RADIO_STATUS_OK) {
        printf("FATAL,uhf_init,st=%d\n", (int)uhf_st);
        while (true) { led_red(); sleep_ms(200); led_off(); sleep_ms(200); }
    }

    radio_status_t sb_st = sx1280f27_init();
    if (sb_st != RADIO_STATUS_OK) {
        printf("FATAL,sband_init,st=%d\n", (int)sb_st);
        while (true) { led_red(); sleep_ms(200); led_off(); sleep_ms(200); }
    }

    sx1280f27_set_profile(DEFAULT_SBAND_PROFILE);
    sx1280f27_set_tx_power_dbm(DEFAULT_SBAND_TX_POWER_DBM);
    rfm98pw_set_tx_power_dbm(DEFAULT_UHF_TX_POWER_DBM);

    led_green();
    sleep_ms(300);
    led_off();

#if IS_FLIGHT
    flight_main();
#elif IS_GROUND
    ground_main();
#else
    #error "Invalid BALLOON_FORCE_ROLE"
#endif

    return 0;
}
