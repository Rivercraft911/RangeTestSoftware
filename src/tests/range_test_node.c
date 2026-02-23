#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hal/board_pins.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/unique_id.h"
#include "protocol/range_packet.h"
#include "radio/radio_rfm98pw.h"
#include "radio/radio_sx1280f27.h"
#include "radio/radio_types.h"
#include "ws2812.pio.h"

// -----------------------------------------------------------------------------
// Build/runtime roles
// -----------------------------------------------------------------------------

enum {
    NODE_ROLE_A = 1,
    NODE_ROLE_B = 2,
    NODE_ROLE_OBSERVER = 3
};

#ifndef RANGE_NODE_FORCE_ROLE
#define RANGE_NODE_FORCE_ROLE 0
#endif

// -----------------------------------------------------------------------------
// Timing/sweep constants
// -----------------------------------------------------------------------------

#define RANGE_TEST_ID_DEFAULT (0x20260221u)
#define WARMUP_MS (2000u)
#define WINDOW_MS (15000u)
#define TX_INTERVAL_MS (25u)
#define RX_REARM_TIMEOUT_MS (250u)
#define HEARTBEAT_PERIOD_MS (1000u)
#define CONFIG_ANNOUNCE_PERIOD_MS (300u)
#define CONTROL_TX_TIMEOUT_MS (1000u)
#define DATA_TX_TIMEOUT_MS (1000u)
#define MAIN_LOOP_SLEEP_MS (2u)
#define SBAND_PHASE_MS (5u * 60u * 1000u)
#define UHF_PHASE_MS (5u * 60u * 1000u)

#define UHF_CONTROL_POWER_DBM (17)
#define SBAND_CONTROL_POWER_DBM (10)

#define HEARTBEAT_PULSE_MS (90u)
#define RX_PACKET_PULSE_MS (80u)
#define ERROR_BLINK_HOLD_MS (2000u)
#define ERROR_BLINK_PERIOD_MS (160u)

#define LISTENER_ROLLUP_EVERY (20u)

typedef enum {
    BAND_SBAND = RANGE_RADIO_SBAND,
    BAND_UHF = RANGE_RADIO_UHF
} band_t;

typedef struct {
    uint8_t cfg_id;
    uint8_t sf;
    uint16_t bw_khz;
    uint8_t cr;
    uint8_t sband_profile_id;
} sband_cfg_t;

static const sband_cfg_t g_sband_cfgs[] = {
    {.cfg_id = 1u, .sf = 7u, .bw_khz = 1625u, .cr = 1u, .sband_profile_id = 1u},
    {.cfg_id = 2u, .sf = 7u, .bw_khz = 812u, .cr = 1u, .sband_profile_id = 2u},
    {.cfg_id = 3u, .sf = 8u, .bw_khz = 406u, .cr = 1u, .sband_profile_id = 3u},
    {.cfg_id = 4u, .sf = 10u, .bw_khz = 203u, .cr = 1u, .sband_profile_id = 4u},
};

static const int8_t g_sband_powers_dbm[] = {-18, -14, -10, -6, -2, 2, 6, 10, 13};
static const int8_t g_uhf_powers_dbm[] = {2, 5, 8, 11, 14, 17, 20};

typedef struct {
    range_sweep_mode_t sweep_mode;
    uint8_t cfg_id;
    uint8_t sf;
    uint16_t bw_khz;
    uint8_t cr;
    int8_t tx_power_dbm;
} sweep_point_t;

#define MAX_SWEEP_POINTS (64u)
static sweep_point_t g_points[MAX_SWEEP_POINTS];
static size_t g_point_count = 0u;
static bool g_sband_ok = false;
static bool g_uhf_ok = false;

// -----------------------------------------------------------------------------
// LED state
// -----------------------------------------------------------------------------

typedef enum {
    LED_ACTIVITY_NONE = 0,
    LED_ACTIVITY_UHF_TX,
    LED_ACTIVITY_UHF_RX,
    LED_ACTIVITY_SBAND_TX,
    LED_ACTIVITY_SBAND_RX
} led_activity_t;

typedef struct {
    PIO pio;
    uint sm;
    led_activity_t activity;
    absolute_time_t heartbeat_pulse_deadline;
    absolute_time_t rx_packet_pulse_deadline;
    led_activity_t rx_packet_pulse_activity;
    absolute_time_t error_blink_deadline;
    absolute_time_t error_blink_toggle_deadline;
    bool error_blink_on;
    uint32_t last_rgb;
} led_state_t;

static led_state_t g_led = {0};

static uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 8) | ((uint32_t)g << 16) | b;
}

static void put_pixel(PIO pio, uint sm, uint32_t pixel_grb) {
    pio_sm_put_blocking(pio, sm, pixel_grb << 8u);
}

static void led_set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t rgb = urgb_u32(r, g, b);
    if (rgb == g_led.last_rgb) {
        return;
    }
    g_led.last_rgb = rgb;
    put_pixel(g_led.pio, g_led.sm, rgb);
}

static void led_set_activity(led_activity_t activity) {
    g_led.activity = activity;
}

static void led_pulse_heartbeat(void) {
    g_led.heartbeat_pulse_deadline = delayed_by_ms(get_absolute_time(), HEARTBEAT_PULSE_MS);
}

static void led_pulse_rx_packet(led_activity_t activity) {
    g_led.rx_packet_pulse_activity = activity;
    g_led.rx_packet_pulse_deadline = delayed_by_ms(get_absolute_time(), RX_PACKET_PULSE_MS);
}

static void led_signal_error(void) {
    g_led.error_blink_deadline = delayed_by_ms(get_absolute_time(), ERROR_BLINK_HOLD_MS);
    g_led.error_blink_toggle_deadline = get_absolute_time();
    g_led.error_blink_on = true;
}

static void led_update(void) {
    absolute_time_t now = get_absolute_time();

    if (absolute_time_diff_us(now, g_led.error_blink_deadline) > 0) {
        if (absolute_time_diff_us(now, g_led.error_blink_toggle_deadline) <= 0) {
            g_led.error_blink_on = !g_led.error_blink_on;
            g_led.error_blink_toggle_deadline = delayed_by_ms(now, ERROR_BLINK_PERIOD_MS);
        }
        if (g_led.error_blink_on) {
            led_set_rgb(48u, 18u, 0u);
        } else {
            led_set_rgb(0u, 0u, 0u);
        }
        return;
    }

    if (absolute_time_diff_us(now, g_led.heartbeat_pulse_deadline) > 0) {
        led_set_rgb(40u, 0u, 0u);
        return;
    }

    if (absolute_time_diff_us(now, g_led.rx_packet_pulse_deadline) > 0) {
        switch (g_led.rx_packet_pulse_activity) {
            case LED_ACTIVITY_UHF_RX:
                led_set_rgb(70u, 0u, 0u);
                return;
            case LED_ACTIVITY_SBAND_RX:
                led_set_rgb(60u, 0u, 76u);
                return;
            default:
                break;
        }
    }

    switch (g_led.activity) {
        case LED_ACTIVITY_UHF_TX:
            led_set_rgb(34u, 34u, 0u); // yellow
            break;
        case LED_ACTIVITY_UHF_RX:
            led_set_rgb(36u, 0u, 0u); // red
            break;
        case LED_ACTIVITY_SBAND_TX:
            led_set_rgb(0u, 0u, 42u); // blue
            break;
        case LED_ACTIVITY_SBAND_RX:
            led_set_rgb(30u, 0u, 34u); // purple
            break;
        case LED_ACTIVITY_NONE:
        default:
            led_set_rgb(0u, 0u, 0u);
            break;
    }
}

static void led_init(void) {
    g_led.pio = pio0;
    g_led.sm = pio_claim_unused_sm(g_led.pio, true);
    uint offset = pio_add_program(g_led.pio, &ws2812_program);
    ws2812_program_init(g_led.pio, g_led.sm, offset, PIN_NEOPIXEL, 800000.0f, false);

    g_led.activity = LED_ACTIVITY_NONE;
    g_led.heartbeat_pulse_deadline = get_absolute_time();
    g_led.rx_packet_pulse_deadline = get_absolute_time();
    g_led.rx_packet_pulse_activity = LED_ACTIVITY_NONE;
    g_led.error_blink_deadline = get_absolute_time();
    g_led.error_blink_toggle_deadline = get_absolute_time();
    g_led.error_blink_on = false;
    g_led.last_rgb = 0xFFFFFFFFu;
    led_update();
}

// -----------------------------------------------------------------------------
// Radio abstraction
// -----------------------------------------------------------------------------

typedef struct {
    uint8_t length;
    uint8_t data[255];
    int16_t rssi_dbm_x100;
    int16_t snr_db_x100;
} raw_frame_t;

static const char *band_str(band_t band) {
    return (band == BAND_SBAND) ? "SBAND" : "UHF";
}

static const char *sweep_mode_str(range_sweep_mode_t mode) {
    return (mode == RANGE_SWEEP_SBAND) ? "SBAND_SWEEP" : "UHF_SWEEP";
}

static const char *dir_str(range_direction_t dir) {
    return (dir == RANGE_DIR_A_TO_B) ? "A_TO_B" : "B_TO_A";
}

static bool band_is_available(band_t band) {
    return (band == BAND_SBAND) ? g_sband_ok : g_uhf_ok;
}

static radio_status_t band_set_profile(band_t band, uint8_t profile_id);
static radio_status_t band_set_power(band_t band, int8_t dbm);
static radio_status_t band_abort(band_t band);

static radio_status_t band_set_profile_retry(band_t band, uint8_t profile_id) {
    radio_status_t st = band_set_profile(band, profile_id);
    if (st == RADIO_STATUS_BUSY) {
        (void)band_abort(band);
        sleep_ms(1);
        st = band_set_profile(band, profile_id);
    }
    return st;
}

static radio_status_t band_set_power_retry(band_t band, int8_t dbm) {
    radio_status_t st = band_set_power(band, dbm);
    if (st == RADIO_STATUS_BUSY) {
        (void)band_abort(band);
        sleep_ms(1);
        st = band_set_power(band, dbm);
    }
    return st;
}

static bool role_is_tx_for_dir(uint8_t role, range_direction_t dir) {
    if (role == NODE_ROLE_A) {
        return dir == RANGE_DIR_A_TO_B;
    }
    if (role == NODE_ROLE_B) {
        return dir == RANGE_DIR_B_TO_A;
    }
    return false;
}

static radio_status_t band_init(band_t band) {
    return (band == BAND_SBAND) ? sx1280f27_init() : rfm98pw_init();
}

static radio_status_t band_set_profile(band_t band, uint8_t profile_id) {
    return (band == BAND_SBAND) ? sx1280f27_set_profile(profile_id) : rfm98pw_set_profile(profile_id);
}

static radio_status_t band_set_power(band_t band, int8_t dbm) {
    return (band == BAND_SBAND) ? sx1280f27_set_tx_power_dbm(dbm) : rfm98pw_set_tx_power_dbm(dbm);
}

static radio_status_t band_start_tx(band_t band, const uint8_t *payload, uint8_t len, uint32_t timeout_ms) {
    return (band == BAND_SBAND)
        ? sx1280f27_start_tx(payload, len, timeout_ms)
        : rfm98pw_start_tx(payload, len, timeout_ms);
}

static radio_status_t band_start_rx(band_t band, uint32_t timeout_ms) {
    return (band == BAND_SBAND) ? sx1280f27_start_rx(timeout_ms) : rfm98pw_start_rx(timeout_ms);
}

static radio_status_t band_poll_event(band_t band, radio_event_t *event) {
    return (band == BAND_SBAND) ? sx1280f27_poll_event(event) : rfm98pw_poll_event(event);
}

static radio_status_t band_abort(band_t band) {
    return (band == BAND_SBAND) ? sx1280f27_abort() : rfm98pw_abort();
}

static radio_status_t band_read_frame(band_t band, raw_frame_t *out) {
    if (out == NULL) {
        return RADIO_STATUS_INVALID_ARG;
    }

    radio_rx_frame_t frame;
    radio_status_t st = (band == BAND_SBAND) ? sx1280f27_read_rx(&frame) : rfm98pw_read_rx(&frame);
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    if (frame.length > sizeof(out->data)) {
        return RADIO_STATUS_INTERNAL_ERROR;
    }

    out->length = frame.length;
    out->rssi_dbm_x100 = frame.rssi_dbm_x100;
    out->snr_db_x100 = frame.snr_db_x100;
    memcpy(out->data, frame.data, frame.length);
    return RADIO_STATUS_OK;
}

static led_activity_t led_activity_for_tx_band(band_t band) {
    return (band == BAND_SBAND) ? LED_ACTIVITY_SBAND_TX : LED_ACTIVITY_UHF_TX;
}

static led_activity_t led_activity_for_rx_band(band_t band) {
    return (band == BAND_SBAND) ? LED_ACTIVITY_SBAND_RX : LED_ACTIVITY_UHF_RX;
}

static bool send_packet_blocking(band_t band, const void *packet, uint8_t len, uint32_t timeout_ms) {
    if ((packet == NULL) || (len == 0u)) {
        return false;
    }

    led_set_activity(led_activity_for_tx_band(band));
    radio_status_t st = band_start_tx(band, (const uint8_t *)packet, len, timeout_ms);
    if (st != RADIO_STATUS_OK) {
        printf("[node] tx start failed band=%s st=%d\n", band_str(band), (int)st);
        led_signal_error();
        led_set_activity(LED_ACTIVITY_NONE);
        return false;
    }

    uint64_t deadline = to_ms_since_boot(get_absolute_time()) + timeout_ms + 100u;
    while (to_ms_since_boot(get_absolute_time()) < deadline) {
        radio_event_t ev = RADIO_EVENT_NONE;
        st = band_poll_event(band, &ev);
        if (st != RADIO_STATUS_OK) {
            printf("[node] tx poll failed band=%s st=%d\n", band_str(band), (int)st);
            led_signal_error();
            (void)band_abort(band);
            led_set_activity(LED_ACTIVITY_NONE);
            return false;
        }
        if (ev == RADIO_EVENT_TX_DONE) {
            led_set_activity(LED_ACTIVITY_NONE);
            return true;
        }
        if ((ev == RADIO_EVENT_TIMEOUT) || (ev == RADIO_EVENT_ERROR)) {
            printf("[node] tx event failure band=%s ev=%d\n", band_str(band), (int)ev);
            led_signal_error();
            (void)band_abort(band);
            led_set_activity(LED_ACTIVITY_NONE);
            return false;
        }
        led_update();
        sleep_ms(1);
    }

    printf("[node] tx timeout band=%s\n", band_str(band));
    led_signal_error();
    (void)band_abort(band);
    led_set_activity(LED_ACTIVITY_NONE);
    return false;
}

// -----------------------------------------------------------------------------
// Protocol helpers
// -----------------------------------------------------------------------------

static void fill_ctrl_header(range_ctrl_header_t *h,
                             uint8_t msg_type,
                             uint8_t role,
                             band_t tested_band,
                             uint32_t test_id,
                             uint32_t window_id,
                             range_sweep_mode_t sweep_mode,
                             uint8_t cfg_id,
                             range_direction_t dir) {
    h->version = RANGE_PACKET_VERSION;
    h->msg_type = msg_type;
    h->role = role;
    h->radio = (uint8_t)tested_band;
    h->test_id = test_id;
    h->window_id = window_id;
    h->sweep_mode = (uint8_t)sweep_mode;
    h->config_id = cfg_id;
    h->direction = (uint8_t)dir;
    h->reserved = 0u;
}

static bool parse_ctrl_header(const uint8_t *data, uint8_t len, range_ctrl_header_t *out) {
    if ((data == NULL) || (out == NULL) || (len < sizeof(range_ctrl_header_t))) {
        return false;
    }

    memcpy(out, data, sizeof(range_ctrl_header_t));
    if (out->version != RANGE_PACKET_VERSION) {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
// Sweep generation
// -----------------------------------------------------------------------------

static bool append_point(const sweep_point_t *p) {
    if (g_point_count >= MAX_SWEEP_POINTS) {
        return false;
    }
    g_points[g_point_count++] = *p;
    return true;
}

static bool build_sweep_points(void) {
    g_point_count = 0u;

    if (g_sband_ok) {
        for (size_t cfg = 0; cfg < (sizeof(g_sband_cfgs) / sizeof(g_sband_cfgs[0])); ++cfg) {
            for (size_t p = 0; p < (sizeof(g_sband_powers_dbm) / sizeof(g_sband_powers_dbm[0])); ++p) {
                sweep_point_t point = {
                    .sweep_mode = RANGE_SWEEP_SBAND,
                    .cfg_id = g_sband_cfgs[cfg].cfg_id,
                    .sf = g_sband_cfgs[cfg].sf,
                    .bw_khz = g_sband_cfgs[cfg].bw_khz,
                    .cr = g_sband_cfgs[cfg].cr,
                    .tx_power_dbm = g_sband_powers_dbm[p]
                };
                if (!append_point(&point)) {
                    return false;
                }
            }
        }
    }

    if (g_uhf_ok) {
        for (size_t p = 0; p < (sizeof(g_uhf_powers_dbm) / sizeof(g_uhf_powers_dbm[0])); ++p) {
            sweep_point_t point = {
                .sweep_mode = RANGE_SWEEP_UHF,
                .cfg_id = 1u,
                .sf = 7u,
                .bw_khz = 125u,
                .cr = 1u,
                .tx_power_dbm = g_uhf_powers_dbm[p]
            };
            if (!append_point(&point)) {
                return false;
            }
        }
    }

    return g_point_count > 0u;
}

static bool has_points_for_mode(range_sweep_mode_t mode) {
    for (size_t i = 0; i < g_point_count; ++i) {
        if (g_points[i].sweep_mode == mode) {
            return true;
        }
    }
    return false;
}

static band_t tested_band_for_mode(range_sweep_mode_t mode) {
    return (mode == RANGE_SWEEP_SBAND) ? BAND_SBAND : BAND_UHF;
}

static band_t control_band_for_mode(range_sweep_mode_t mode) {
    band_t preferred = (mode == RANGE_SWEEP_SBAND) ? BAND_UHF : BAND_SBAND;
    band_t tested = tested_band_for_mode(mode);
    if (band_is_available(preferred)) {
        return preferred;
    }
    if (band_is_available(tested)) {
        return tested;
    }
    return preferred;
}

static uint8_t sband_profile_from_cfg(uint8_t cfg_id) {
    for (size_t i = 0; i < (sizeof(g_sband_cfgs) / sizeof(g_sband_cfgs[0])); ++i) {
        if (g_sband_cfgs[i].cfg_id == cfg_id) {
            return g_sband_cfgs[i].sband_profile_id;
        }
    }
    return 1u;
}

static void quiesce_radios_for_reconfig(void) {
    // Profile/power setters require radios not to be in active TX/RX wait states.
    // Always force both links back to IDLE before applying a new sweep point.
    if (g_sband_ok) {
        (void)band_abort(BAND_SBAND);
    }
    if (g_uhf_ok) {
        (void)band_abort(BAND_UHF);
    }
}

static bool configure_radios_for_point(const sweep_point_t *point) {
    band_t tested = tested_band_for_mode(point->sweep_mode);
    band_t control = control_band_for_mode(point->sweep_mode);
    static uint32_t unsupported_log_count = 0u;

    radio_status_t st;

    if (!band_is_available(tested) || !band_is_available(control)) {
        if ((unsupported_log_count++ % 16u) == 0u) {
            printf("[node] skip point mode=%s cfg=%u: tested=%s(%s) control=%s(%s)\n",
                   sweep_mode_str(point->sweep_mode),
                   (unsigned)point->cfg_id,
                   band_str(tested),
                   band_is_available(tested) ? "ok" : "down",
                   band_str(control),
                   band_is_available(control) ? "ok" : "down");
        }
        return false;
    }

    quiesce_radios_for_reconfig();

    // Configure tested band
    if (tested == BAND_SBAND) {
        st = band_set_profile_retry(BAND_SBAND, sband_profile_from_cfg(point->cfg_id));
        if (st != RADIO_STATUS_OK) {
            printf("[node] sband profile set failed st=%d\n", (int)st);
            return false;
        }
        st = band_set_power_retry(BAND_SBAND, point->tx_power_dbm);
        if (st != RADIO_STATUS_OK) {
            printf("[node] sband power set failed st=%d\n", (int)st);
            return false;
        }
    } else {
        st = band_set_profile_retry(BAND_UHF, 1u);
        if (st != RADIO_STATUS_OK) {
            printf("[node] uhf profile set failed st=%d\n", (int)st);
            return false;
        }
        st = band_set_power_retry(BAND_UHF, point->tx_power_dbm);
        if (st != RADIO_STATUS_OK) {
            printf("[node] uhf power set failed st=%d\n", (int)st);
            return false;
        }
    }

    // In single-band fallback mode, keep tested-band sweep settings intact.
    if (control == tested) {
        static uint32_t same_band_notice_count = 0u;
        if ((same_band_notice_count++ % 16u) == 0u) {
            printf("[node] control fallback: using %s for both data/control\n", band_str(control));
        }
        return true;
    }

    // Configure control band as fixed telemetry link.
    if (control == BAND_SBAND) {
        st = band_set_profile_retry(BAND_SBAND, 1u);
        if (st != RADIO_STATUS_OK) {
            printf("[node] sband control profile set failed st=%d\n", (int)st);
            return false;
        }
        st = band_set_power_retry(BAND_SBAND, SBAND_CONTROL_POWER_DBM);
        if (st != RADIO_STATUS_OK) {
            printf("[node] sband control power set failed st=%d\n", (int)st);
            return false;
        }
    } else {
        st = band_set_profile_retry(BAND_UHF, 1u);
        if (st != RADIO_STATUS_OK) {
            printf("[node] uhf control profile set failed st=%d\n", (int)st);
            return false;
        }
        st = band_set_power_retry(BAND_UHF, UHF_CONTROL_POWER_DBM);
        if (st != RADIO_STATUS_OK) {
            printf("[node] uhf control power set failed st=%d\n", (int)st);
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------
// Window run helpers
// -----------------------------------------------------------------------------

typedef struct {
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t crc_error_count;
    int64_t rssi_sum_x100;
    int64_t snr_sum_x100;
    uint32_t metrics_count;
} window_stats_t;

static void window_stats_reset(window_stats_t *s) {
    memset(s, 0, sizeof(*s));
}

static bool run_tx_window(band_t tested_band,
                          uint8_t node_role,
                          const sweep_point_t *point,
                          uint32_t test_id,
                          uint32_t window_id,
                          range_direction_t dir,
                          uint16_t expected_packets,
                          window_stats_t *stats) {
    uint64_t start_ms = to_ms_since_boot(get_absolute_time());
    uint64_t end_ms = start_ms + WINDOW_MS;
    uint64_t next_tx_ms = start_ms;
    bool op_active = false;

    window_stats_reset(stats);
    led_set_activity(led_activity_for_tx_band(tested_band));

    while ((to_ms_since_boot(get_absolute_time()) < end_ms) || op_active) {
        uint64_t now_ms = to_ms_since_boot(get_absolute_time());

        if (!op_active && (now_ms < end_ms) && (now_ms >= next_tx_ms)) {
            range_window_data_t pkt;
            fill_ctrl_header(&pkt.h,
                             RANGE_MSG_WINDOW_DATA,
                             node_role,
                             tested_band,
                             test_id,
                             window_id,
                             point->sweep_mode,
                             point->cfg_id,
                             dir);
            pkt.seq = stats->tx_count;
            pkt.tx_uptime_ms = (uint32_t)now_ms;
            pkt.tx_power_dbm = point->tx_power_dbm;
            pkt.rf_profile = (tested_band == BAND_SBAND) ? sband_profile_from_cfg(point->cfg_id) : 1u;

            radio_status_t st = band_start_tx(tested_band, (const uint8_t *)&pkt, (uint8_t)sizeof(pkt), DATA_TX_TIMEOUT_MS);
            if (st == RADIO_STATUS_OK) {
                op_active = true;
                stats->tx_count++;
            } else {
                printf("[node] data tx start failed band=%s st=%d\n", band_str(tested_band), (int)st);
                led_signal_error();
            }
            next_tx_ms += TX_INTERVAL_MS;
        }

        if (op_active) {
            radio_event_t ev = RADIO_EVENT_NONE;
            radio_status_t st = band_poll_event(tested_band, &ev);
            if (st != RADIO_STATUS_OK) {
                printf("[node] data tx poll failed band=%s st=%d\n", band_str(tested_band), (int)st);
                led_signal_error();
                (void)band_abort(tested_band);
                op_active = false;
            } else if ((ev == RADIO_EVENT_TX_DONE) || (ev == RADIO_EVENT_TIMEOUT) || (ev == RADIO_EVENT_ERROR)) {
                if (ev != RADIO_EVENT_TX_DONE) {
                    led_signal_error();
                }
                op_active = false;
            }
        }

        led_update();
        sleep_ms(1);
    }

    (void)band_abort(tested_band);
    led_set_activity(LED_ACTIVITY_NONE);

    // Expected packets for summary consistency.
    if (stats->tx_count < expected_packets) {
        stats->tx_count = expected_packets;
    }
    return true;
}

static bool run_rx_window(band_t tested_band,
                          const sweep_point_t *point,
                          uint32_t test_id,
                          uint32_t window_id,
                          range_direction_t dir,
                          uint16_t expected_packets,
                          window_stats_t *stats) {
    uint64_t start_ms = to_ms_since_boot(get_absolute_time());
    uint64_t end_ms = start_ms + WINDOW_MS;
    bool rx_active = false;

    window_stats_reset(stats);
    stats->tx_count = expected_packets;
    led_set_activity(led_activity_for_rx_band(tested_band));

    while (to_ms_since_boot(get_absolute_time()) < end_ms) {
        if (!rx_active) {
            radio_status_t st = band_start_rx(tested_band, RX_REARM_TIMEOUT_MS);
            if (st == RADIO_STATUS_OK) {
                rx_active = true;
            } else {
                printf("[node] rx arm failed band=%s st=%d\n", band_str(tested_band), (int)st);
                led_signal_error();
                sleep_ms(4);
            }
        }

        if (rx_active) {
            radio_event_t ev = RADIO_EVENT_NONE;
            radio_status_t st = band_poll_event(tested_band, &ev);
            if (st != RADIO_STATUS_OK) {
                printf("[node] rx poll failed band=%s st=%d\n", band_str(tested_band), (int)st);
                led_signal_error();
                rx_active = false;
                (void)band_abort(tested_band);
            } else if (ev == RADIO_EVENT_RX_DONE) {
                raw_frame_t frame;
                st = band_read_frame(tested_band, &frame);
                if (st == RADIO_STATUS_OK && frame.length >= sizeof(range_ctrl_header_t)) {
                    range_ctrl_header_t h;
                    if (parse_ctrl_header(frame.data, frame.length, &h) &&
                        h.msg_type == RANGE_MSG_WINDOW_DATA &&
                        h.test_id == test_id &&
                        h.window_id == window_id &&
                        h.sweep_mode == (uint8_t)point->sweep_mode &&
                        h.config_id == point->cfg_id &&
                        h.direction == (uint8_t)dir) {
                        stats->rx_count++;
                        stats->rssi_sum_x100 += frame.rssi_dbm_x100;
                        stats->snr_sum_x100 += frame.snr_db_x100;
                        stats->metrics_count++;
                        led_pulse_rx_packet(led_activity_for_rx_band(tested_band));
                        if ((stats->rx_count == 1u) || ((stats->rx_count % 25u) == 0u)) {
                            printf("[node] rx packet band=%s count=%" PRIu32 " len=%u rssi=%.2f snr=%.2f\n",
                                   band_str(tested_band),
                                   stats->rx_count,
                                   (unsigned)frame.length,
                                   (double)frame.rssi_dbm_x100 / 100.0,
                                   (double)frame.snr_db_x100 / 100.0);
                        }
                    }
                }
                rx_active = false;
            } else if (ev == RADIO_EVENT_CRC_FAIL) {
                stats->crc_error_count++;
                rx_active = false;
            } else if ((ev == RADIO_EVENT_TIMEOUT) || (ev == RADIO_EVENT_ERROR)) {
                if (ev == RADIO_EVENT_ERROR) {
                    led_signal_error();
                }
                rx_active = false;
            }
        }

        led_update();
        sleep_ms(1);
    }

    (void)band_abort(tested_band);
    led_set_activity(LED_ACTIVITY_NONE);
    return true;
}

// -----------------------------------------------------------------------------
// Observer dual-RX helper
// -----------------------------------------------------------------------------

typedef struct {
    bool sband_armed;
    bool uhf_armed;
} dual_rx_ctx_t;

static void dual_rx_init(dual_rx_ctx_t *ctx) {
    ctx->sband_armed = false;
    ctx->uhf_armed = false;
}

static bool dual_rx_poll(dual_rx_ctx_t *ctx, band_t *rx_band, raw_frame_t *out_frame) {
    if (g_sband_ok && !ctx->sband_armed) {
        if (band_start_rx(BAND_SBAND, RX_REARM_TIMEOUT_MS) == RADIO_STATUS_OK) {
            ctx->sband_armed = true;
        }
    }
    if (g_uhf_ok && !ctx->uhf_armed) {
        if (band_start_rx(BAND_UHF, RX_REARM_TIMEOUT_MS) == RADIO_STATUS_OK) {
            ctx->uhf_armed = true;
        }
    }

    if (ctx->sband_armed) {
        radio_event_t ev = RADIO_EVENT_NONE;
        radio_status_t st = band_poll_event(BAND_SBAND, &ev);
        if (st == RADIO_STATUS_OK) {
            if (ev == RADIO_EVENT_RX_DONE) {
                if (band_read_frame(BAND_SBAND, out_frame) == RADIO_STATUS_OK) {
                    *rx_band = BAND_SBAND;
                    ctx->sband_armed = false;
                    led_pulse_rx_packet(LED_ACTIVITY_SBAND_RX);
                    return true;
                }
                ctx->sband_armed = false;
            } else if ((ev == RADIO_EVENT_TIMEOUT) || (ev == RADIO_EVENT_CRC_FAIL) || (ev == RADIO_EVENT_ERROR)) {
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
                    led_pulse_rx_packet(LED_ACTIVITY_UHF_RX);
                    return true;
                }
                ctx->uhf_armed = false;
            } else if ((ev == RADIO_EVENT_TIMEOUT) || (ev == RADIO_EVENT_CRC_FAIL) || (ev == RADIO_EVENT_ERROR)) {
                ctx->uhf_armed = false;
            }
        } else {
            ctx->uhf_armed = false;
        }
    }

    return false;
}

// -----------------------------------------------------------------------------
// Role selection
// -----------------------------------------------------------------------------

static uint8_t detect_node_role(void) {
#if RANGE_NODE_FORCE_ROLE == NODE_ROLE_A
    return NODE_ROLE_A;
#elif RANGE_NODE_FORCE_ROLE == NODE_ROLE_B
    return NODE_ROLE_B;
#elif RANGE_NODE_FORCE_ROLE == NODE_ROLE_OBSERVER
    return NODE_ROLE_OBSERVER;
#else
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    return (id.id[7] & 0x01u) ? NODE_ROLE_A : NODE_ROLE_B;
#endif
}

// -----------------------------------------------------------------------------
// Master/follower protocol flow
// -----------------------------------------------------------------------------

typedef struct {
    uint32_t test_id;
    uint32_t window_id;
    uint16_t announce_seq;
    uint32_t heartbeat_next_ms;
} node_session_t;

static void send_heartbeat(band_t control_band,
                           uint8_t role,
                           const sweep_point_t *point,
                           uint32_t test_id,
                           uint32_t window_id,
                           uint16_t status_flags) {
    range_heartbeat_t hb;
    fill_ctrl_header(&hb.h,
                     RANGE_MSG_HEARTBEAT,
                     role,
                     tested_band_for_mode(point->sweep_mode),
                     test_id,
                     window_id,
                     point->sweep_mode,
                     point->cfg_id,
                     RANGE_DIR_A_TO_B);
    hb.uptime_ms = (uint32_t)to_ms_since_boot(get_absolute_time());
    hb.last_window_seen = window_id;
    hb.status_flags = status_flags;
    (void)send_packet_blocking(control_band, &hb, (uint8_t)sizeof(hb), CONTROL_TX_TIMEOUT_MS);
}

static void send_config_announce(band_t control_band,
                                 uint8_t role,
                                 const sweep_point_t *point,
                                 uint32_t test_id,
                                 uint32_t window_id,
                                 range_direction_t dir,
                                 uint16_t announce_seq,
                                 uint16_t expected_packets) {
    range_config_announce_t pkt;
    fill_ctrl_header(&pkt.h,
                     RANGE_MSG_CONFIG_ANNOUNCE,
                     role,
                     tested_band_for_mode(point->sweep_mode),
                     test_id,
                     window_id,
                     point->sweep_mode,
                     point->cfg_id,
                     dir);
    pkt.tx_power_dbm = point->tx_power_dbm;
    pkt.sf = point->sf;
    pkt.bw_khz = point->bw_khz;
    pkt.cr = point->cr;
    pkt.interval_ms = TX_INTERVAL_MS;
    pkt.warmup_ms = WARMUP_MS;
    pkt.window_ms = WINDOW_MS;
    pkt.expected_packets = expected_packets;
    pkt.announce_seq = announce_seq;

    (void)send_packet_blocking(control_band, &pkt, (uint8_t)sizeof(pkt), CONTROL_TX_TIMEOUT_MS);
}

static void send_summary(band_t control_band,
                         uint8_t role,
                         const sweep_point_t *point,
                         uint32_t test_id,
                         uint32_t window_id,
                         range_direction_t dir,
                         const window_stats_t *stats) {
    if (stats->rx_count == 0u) {
        // Only emit summary frames when peer traffic is actually observed.
        // This keeps listener output meaningful for "both boards active" testing.
        printf("[node] summary suppressed mode=%s cfg=%u dir=%s pwr=%d (rx=0)\n",
               sweep_mode_str(point->sweep_mode),
               (unsigned)point->cfg_id,
               dir_str(dir),
               (int)point->tx_power_dbm);
        return;
    }

    range_window_summary_t s;
    fill_ctrl_header(&s.h,
                     RANGE_MSG_WINDOW_SUMMARY,
                     role,
                     tested_band_for_mode(point->sweep_mode),
                     test_id,
                     window_id,
                     point->sweep_mode,
                     point->cfg_id,
                     dir);

    s.tx_power_dbm = point->tx_power_dbm;
    s.sf = point->sf;
    s.bw_khz = point->bw_khz;
    s.cr = point->cr;
    s.tx_count = stats->tx_count;
    s.rx_count = stats->rx_count;
    s.crc_error_count = stats->crc_error_count;

    if (stats->tx_count > 0u) {
        uint32_t scaled = (uint32_t)((stats->rx_count * 10000u) / stats->tx_count);
        if (scaled > 10000u) {
            scaled = 10000u;
        }
        s.prr_x100 = (uint16_t)scaled;
        s.drop_x100 = (uint16_t)(10000u - scaled);
    } else {
        s.prr_x100 = 0u;
        s.drop_x100 = 10000u;
    }

    s.goodput_bps = (uint32_t)((stats->rx_count * (uint32_t)sizeof(range_window_data_t) * 8u * 1000u) / WINDOW_MS);

    if (stats->metrics_count > 0u) {
        s.rssi_mean_x100 = (int16_t)(stats->rssi_sum_x100 / (int64_t)stats->metrics_count);
        s.snr_mean_x100 = (int16_t)(stats->snr_sum_x100 / (int64_t)stats->metrics_count);
    } else {
        s.rssi_mean_x100 = RANGE_INVALID_RSSI_DBM_X100;
        s.snr_mean_x100 = RANGE_INVALID_SNR_DB_X100;
    }

    printf("[node] summary mode=%s cfg=%u dir=%s pwr=%d tx=%" PRIu32 " rx=%" PRIu32 " prr=%.2f%%\n",
           sweep_mode_str(point->sweep_mode),
           (unsigned)point->cfg_id,
           dir_str(dir),
           (int)s.tx_power_dbm,
           s.tx_count,
           s.rx_count,
           (double)s.prr_x100 / 100.0);

    (void)send_packet_blocking(control_band, &s, (uint8_t)sizeof(s), CONTROL_TX_TIMEOUT_MS);
}

static bool run_master_mode_phase(uint8_t role,
                                  node_session_t *session,
                                  range_sweep_mode_t mode,
                                  uint32_t phase_duration_ms) {
    uint16_t expected_packets = (uint16_t)(WINDOW_MS / TX_INTERVAL_MS);
    uint64_t phase_start_ms = to_ms_since_boot(get_absolute_time());
    uint64_t phase_end_ms = phase_start_ms + phase_duration_ms;
    size_t point_cursor = 0u;

    printf("[node] phase start mode=%s duration_s=%u\n",
           sweep_mode_str(mode),
           (unsigned)(phase_duration_ms / 1000u));

    while (to_ms_since_boot(get_absolute_time()) < phase_end_ms) {
        const sweep_point_t *point = NULL;
        size_t point_index = 0u;
        for (size_t step = 0; step < g_point_count; ++step) {
            size_t idx = (point_cursor + step) % g_point_count;
            if (g_points[idx].sweep_mode == mode) {
                point = &g_points[idx];
                point_index = idx;
                point_cursor = (idx + 1u) % g_point_count;
                break;
            }
        }

        if (point == NULL) {
            // No points for this mode.
            break;
        }

        band_t tested = tested_band_for_mode(point->sweep_mode);
        band_t control = control_band_for_mode(point->sweep_mode);

        if (!configure_radios_for_point(point)) {
            if (!band_is_available(tested) || !band_is_available(control)) {
                continue;
            }
            led_signal_error();
            return false;
        }

        for (int dir_idx = 0; dir_idx < 2; ++dir_idx) {
            if (to_ms_since_boot(get_absolute_time()) >= phase_end_ms) {
                break;
            }

            range_direction_t dir = (dir_idx == 0) ? RANGE_DIR_A_TO_B : RANGE_DIR_B_TO_A;
            bool this_node_tx = role_is_tx_for_dir(role, dir);

            session->window_id++;
            uint64_t warmup_deadline = to_ms_since_boot(get_absolute_time()) + WARMUP_MS;
            uint64_t next_cfg_ms = to_ms_since_boot(get_absolute_time());

            printf("[node] phase APPLY mode=%s point=%zu/%zu cfg=%u dir=%s pwr=%d role=%s\n",
                   sweep_mode_str(point->sweep_mode),
                   point_index + 1u,
                   g_point_count,
                   (unsigned)point->cfg_id,
                   dir_str(dir),
                   (int)point->tx_power_dbm,
                   this_node_tx ? "TX" : "RX");

            while (to_ms_since_boot(get_absolute_time()) < warmup_deadline) {
                uint64_t now = to_ms_since_boot(get_absolute_time());
                if (now >= next_cfg_ms) {
                    send_config_announce(control,
                                         role,
                                         point,
                                         session->test_id,
                                         session->window_id,
                                         dir,
                                         session->announce_seq++,
                                         expected_packets);
                    next_cfg_ms += CONFIG_ANNOUNCE_PERIOD_MS;
                }

                if (now >= session->heartbeat_next_ms) {
                    send_heartbeat(control, role, point, session->test_id, session->window_id, RANGE_STATUS_OK);
                    session->heartbeat_next_ms = now + HEARTBEAT_PERIOD_MS;
                }

                led_update();
                sleep_ms(MAIN_LOOP_SLEEP_MS);
            }

            window_stats_t stats;
            if (this_node_tx) {
                (void)run_tx_window(tested,
                                    role,
                                    point,
                                    session->test_id,
                                    session->window_id,
                                    dir,
                                    expected_packets,
                                    &stats);
            } else {
                (void)run_rx_window(tested,
                                    point,
                                    session->test_id,
                                    session->window_id,
                                    dir,
                                    expected_packets,
                                    &stats);
                send_summary(control,
                             role,
                             point,
                             session->test_id,
                             session->window_id,
                             dir,
                             &stats);
            }

            led_set_activity(LED_ACTIVITY_NONE);
        }
    }

    printf("[node] phase done mode=%s elapsed_s=%u\n",
           sweep_mode_str(mode),
           (unsigned)((to_ms_since_boot(get_absolute_time()) - phase_start_ms) / 1000u));
    return true;
}

static bool run_master_session(uint8_t role, node_session_t *session) {
    if (has_points_for_mode(RANGE_SWEEP_SBAND)) {
        if (!run_master_mode_phase(role, session, RANGE_SWEEP_SBAND, SBAND_PHASE_MS)) {
            return false;
        }
    }

    if (has_points_for_mode(RANGE_SWEEP_UHF)) {
        if (!run_master_mode_phase(role, session, RANGE_SWEEP_UHF, UHF_PHASE_MS)) {
            return false;
        }
    }

    return true;
}

static bool try_parse_announce(const raw_frame_t *frame, range_config_announce_t *out) {
    if ((frame == NULL) || (out == NULL)) {
        return false;
    }
    if (frame->length < sizeof(range_config_announce_t)) {
        return false;
    }

    memcpy(out, frame->data, sizeof(range_config_announce_t));
    return (out->h.version == RANGE_PACKET_VERSION) && (out->h.msg_type == RANGE_MSG_CONFIG_ANNOUNCE);
}

static bool run_follower_session(uint8_t role, node_session_t *session) {
    dual_rx_ctx_t ctrl_rx;
    dual_rx_init(&ctrl_rx);

    while (true) {
        band_t rx_band = BAND_UHF;
        raw_frame_t frame;
        if (!dual_rx_poll(&ctrl_rx, &rx_band, &frame)) {
            led_update();
            sleep_ms(MAIN_LOOP_SLEEP_MS);
            continue;
        }

        led_set_activity(led_activity_for_rx_band(rx_band));

        range_ctrl_header_t h;
        if (!parse_ctrl_header(frame.data, frame.length, &h)) {
            continue;
        }

        if (h.msg_type == RANGE_MSG_HEARTBEAT) {
            led_pulse_heartbeat();
            continue;
        }

        if (h.msg_type != RANGE_MSG_CONFIG_ANNOUNCE) {
            continue;
        }

        range_config_announce_t ann;
        if (!try_parse_announce(&frame, &ann)) {
            continue;
        }

        sweep_point_t point = {
            .sweep_mode = (range_sweep_mode_t)ann.h.sweep_mode,
            .cfg_id = ann.h.config_id,
            .sf = ann.sf,
            .bw_khz = ann.bw_khz,
            .cr = ann.cr,
            .tx_power_dbm = ann.tx_power_dbm
        };

        band_t tested = tested_band_for_mode(point.sweep_mode);
        band_t control = control_band_for_mode(point.sweep_mode);
        if (!band_is_available(tested) || !band_is_available(control)) {
            static uint32_t skip_cfg_logs = 0u;
            if ((skip_cfg_logs++ % 16u) == 0u) {
                printf("[node] ignore config mode=%s cfg=%u: tested=%s(%s) control=%s(%s)\n",
                       sweep_mode_str(point.sweep_mode),
                       (unsigned)point.cfg_id,
                       band_str(tested),
                       band_is_available(tested) ? "ok" : "down",
                       band_str(control),
                       band_is_available(control) ? "ok" : "down");
            }
            continue;
        }

        if (!configure_radios_for_point(&point)) {
            led_signal_error();
            dual_rx_init(&ctrl_rx);
            continue;
        }

        session->test_id = ann.h.test_id;
        session->window_id = ann.h.window_id;

        uint64_t start_ms = to_ms_since_boot(get_absolute_time()) + ann.warmup_ms;
        range_direction_t dir = (range_direction_t)ann.h.direction;
        bool this_node_tx = role_is_tx_for_dir(role, dir);

        printf("[node] follower sync mode=%s cfg=%u dir=%s pwr=%d role=%s\n",
               sweep_mode_str(point.sweep_mode),
               (unsigned)point.cfg_id,
               dir_str(dir),
               (int)point.tx_power_dbm,
               this_node_tx ? "TX" : "RX");

        while (to_ms_since_boot(get_absolute_time()) < start_ms) {
            led_update();
            sleep_ms(MAIN_LOOP_SLEEP_MS);
        }

        window_stats_t stats;
        tested = tested_band_for_mode(point.sweep_mode);
        control = control_band_for_mode(point.sweep_mode);

        if (this_node_tx) {
            (void)run_tx_window(tested,
                                role,
                                &point,
                                session->test_id,
                                session->window_id,
                                dir,
                                ann.expected_packets,
                                &stats);
        } else {
            (void)run_rx_window(tested,
                                &point,
                                session->test_id,
                                session->window_id,
                                dir,
                                ann.expected_packets,
                                &stats);
            send_summary(control,
                         role,
                         &point,
                         session->test_id,
                         session->window_id,
                         dir,
                         &stats);
        }

        led_set_activity(LED_ACTIVITY_NONE);
        dual_rx_init(&ctrl_rx);
    }
}

static void print_observer_line_summary(const range_window_summary_t *s) {
    printf("SUMMARY,test_id=%" PRIu32 ",window_id=%" PRIu32 ",band=%s,mode=%s,cfg=%u,dir=%s,pwr_dbm=%d,tx=%" PRIu32 ",rx=%" PRIu32 ",crc=%" PRIu32 ",prr_x100=%u,drop_x100=%u,goodput_bps=%" PRIu32 ",rssi_mean_x100=%d,snr_mean_x100=%d\n",
           s->h.test_id,
           s->h.window_id,
           ((band_t)s->h.radio == BAND_SBAND) ? "SBAND" : "UHF",
           ((range_sweep_mode_t)s->h.sweep_mode == RANGE_SWEEP_SBAND) ? "SBAND_SWEEP" : "UHF_SWEEP",
           (unsigned)s->h.config_id,
           dir_str((range_direction_t)s->h.direction),
           (int)s->tx_power_dbm,
           s->tx_count,
           s->rx_count,
           s->crc_error_count,
           (unsigned)s->prr_x100,
           (unsigned)s->drop_x100,
           s->goodput_bps,
           (int)s->rssi_mean_x100,
           (int)s->snr_mean_x100);
}

static void print_observer_line_config(const range_config_announce_t *c) {
    printf("CONFIG,test_id=%" PRIu32 ",window_id=%" PRIu32 ",band=%s,mode=%s,cfg=%u,dir=%s,pwr_dbm=%d,sf=%u,bw_khz=%u,cr=%u,interval_ms=%u,warmup_ms=%u,window_ms=%u,expected=%u,seq=%u\n",
           c->h.test_id,
           c->h.window_id,
           ((band_t)c->h.radio == BAND_SBAND) ? "SBAND" : "UHF",
           ((range_sweep_mode_t)c->h.sweep_mode == RANGE_SWEEP_SBAND) ? "SBAND_SWEEP" : "UHF_SWEEP",
           (unsigned)c->h.config_id,
           dir_str((range_direction_t)c->h.direction),
           (int)c->tx_power_dbm,
           (unsigned)c->sf,
           (unsigned)c->bw_khz,
           (unsigned)c->cr,
           (unsigned)c->interval_ms,
           (unsigned)c->warmup_ms,
           (unsigned)c->window_ms,
           (unsigned)c->expected_packets,
           (unsigned)c->announce_seq);
}

static void print_observer_line_heartbeat(const range_heartbeat_t *h) {
    const char *role_str = "UNK";
    if (h->h.role == NODE_ROLE_A) {
        role_str = "A";
    } else if (h->h.role == NODE_ROLE_B) {
        role_str = "B";
    } else if (h->h.role == NODE_ROLE_OBSERVER) {
        role_str = "OBS";
    }

    printf("HEARTBEAT,test_id=%" PRIu32 ",window_id=%" PRIu32 ",from=%s,band=%s,mode=%s,cfg=%u,uptime_ms=%" PRIu32 ",status=0x%04X\n",
           h->h.test_id,
           h->h.window_id,
           role_str,
           ((band_t)h->h.radio == BAND_SBAND) ? "SBAND" : "UHF",
           ((range_sweep_mode_t)h->h.sweep_mode == RANGE_SWEEP_SBAND) ? "SBAND_SWEEP" : "UHF_SWEEP",
           (unsigned)h->h.config_id,
           h->uptime_ms,
           (unsigned)h->status_flags);
}

static bool run_observer_loop(void) {
    radio_status_t st;

    if (g_sband_ok) {
        st = band_set_profile(BAND_SBAND, 1u);
        if (st != RADIO_STATUS_OK) {
            printf("[observer] sband profile set failed st=%d\n", (int)st);
            g_sband_ok = false;
        }
    }
    if (g_uhf_ok) {
        st = band_set_profile(BAND_UHF, 1u);
        if (st != RADIO_STATUS_OK) {
            printf("[observer] uhf profile set failed st=%d\n", (int)st);
            g_uhf_ok = false;
        }
    }
    if (!g_sband_ok && !g_uhf_ok) {
        printf("[observer] no radios available\n");
        return false;
    }

    dual_rx_ctx_t rx;
    dual_rx_init(&rx);

    uint32_t seen_summary = 0u;
    while (true) {
        band_t rx_band;
        raw_frame_t frame;
        if (!dual_rx_poll(&rx, &rx_band, &frame)) {
            led_update();
            sleep_ms(MAIN_LOOP_SLEEP_MS);
            continue;
        }

        led_set_activity(led_activity_for_rx_band(rx_band));

        range_ctrl_header_t h;
        if (!parse_ctrl_header(frame.data, frame.length, &h)) {
            continue;
        }

        if ((h.msg_type == RANGE_MSG_HEARTBEAT) && (frame.length >= sizeof(range_heartbeat_t))) {
            range_heartbeat_t hb;
            memcpy(&hb, frame.data, sizeof(hb));
            print_observer_line_heartbeat(&hb);
            led_pulse_heartbeat();
        } else if ((h.msg_type == RANGE_MSG_CONFIG_ANNOUNCE) && (frame.length >= sizeof(range_config_announce_t))) {
            range_config_announce_t cfg;
            memcpy(&cfg, frame.data, sizeof(cfg));
            print_observer_line_config(&cfg);
        } else if ((h.msg_type == RANGE_MSG_WINDOW_SUMMARY) && (frame.length >= sizeof(range_window_summary_t))) {
            range_window_summary_t sum;
            memcpy(&sum, frame.data, sizeof(sum));
            print_observer_line_summary(&sum);
            ++seen_summary;
            if ((seen_summary % LISTENER_ROLLUP_EVERY) == 0u) {
                printf("ROLLUP,seen_summaries=%" PRIu32 "\n", seen_summary);
            }
        }

        led_update();
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(1200);

    board_pins_init();
    led_init();

    uint8_t role = detect_node_role();
    printf("RangeTestNode start role=%u\n", (unsigned)role);

    radio_status_t st;
    st = band_init(BAND_SBAND);
    g_sband_ok = (st == RADIO_STATUS_OK);
    if (!g_sband_ok) {
        printf("[node] sband init failed st=%d\n", (int)st);
        led_signal_error();
    }
    st = band_init(BAND_UHF);
    g_uhf_ok = (st == RADIO_STATUS_OK);
    if (!g_uhf_ok) {
        printf("[node] uhf init failed st=%d\n", (int)st);
        led_signal_error();
    }

    if (!build_sweep_points()) {
        printf("[node] failed to build sweep matrix (sband=%s uhf=%s)\n",
               g_sband_ok ? "up" : "down",
               g_uhf_ok ? "up" : "down");
        led_signal_error();
        while (true) {
            led_update();
            sleep_ms(10);
        }
    }
    printf("[node] radios: sband=%s uhf=%s points=%u\n",
           g_sband_ok ? "up" : "down",
           g_uhf_ok ? "up" : "down",
           (unsigned)g_point_count);

    node_session_t session = {
        .test_id = RANGE_TEST_ID_DEFAULT,
        .window_id = 0u,
        .announce_seq = 1u,
        .heartbeat_next_ms = to_ms_since_boot(get_absolute_time())
    };

    if (role == NODE_ROLE_OBSERVER) {
        printf("[observer] running dual-band listener mode\n");
        if (!run_observer_loop()) {
            led_signal_error();
            while (true) {
                led_update();
                sleep_ms(10);
            }
        }
        return 0;
    }

    if (role == NODE_ROLE_A) {
        printf("[node] role A (master)\n");
        if (!run_master_session(role, &session)) {
            printf("[node] master session failed\n");
            led_signal_error();
        }
    } else {
        printf("[node] role B (follower)\n");
        if (!run_follower_session(role, &session)) {
            printf("[node] follower session failed\n");
            led_signal_error();
        }
    }

    while (true) {
        led_update();
        sleep_ms(10);
    }
}
