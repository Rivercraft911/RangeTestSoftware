#ifndef RANGE_PACKET_H
#define RANGE_PACKET_H

#include <limits.h>
#include <stdint.h>

#define RANGE_PACKET_VERSION 1u

typedef enum {
    RANGE_MSG_TRANSMIT = 1u,
    RANGE_MSG_RECEIVE = 2u,
    RANGE_MSG_HEARTBEAT = 3u,
    RANGE_MSG_WINDOW_SUMMARY = 4u,
    RANGE_MSG_CONFIG_ANNOUNCE = 5u,
    RANGE_MSG_WINDOW_DATA = 6u
} range_msg_type_t;

typedef enum {
    RANGE_ROLE_TRANSMITTER = 1u,
    RANGE_ROLE_RECEIVER = 2u,
    RANGE_ROLE_OBSERVER = 3u
} range_role_t;

typedef enum {
    RANGE_RADIO_UHF = 1u,
    RANGE_RADIO_SBAND = 2u
} range_radio_t;

typedef enum {
    RANGE_SWEEP_SBAND = 1u,
    RANGE_SWEEP_UHF = 2u
} range_sweep_mode_t;

typedef enum {
    RANGE_DIR_A_TO_B = 1u,
    RANGE_DIR_B_TO_A = 2u
} range_direction_t;

enum {
    RANGE_STATUS_OK = 0u,
    RANGE_STATUS_TIMEOUT = (1u << 0),
    RANGE_STATUS_CRC_FAIL = (1u << 1),
    RANGE_STATUS_RADIO_ERROR = (1u << 2)
};

#define RANGE_INVALID_RSSI_DBM_X100 INT16_MIN
#define RANGE_INVALID_SNR_DB_X100 INT16_MIN

typedef struct __attribute__((packed)) {
    uint8_t version;
    uint8_t msg_type;
    uint8_t role;
    uint8_t radio;
    uint32_t test_id;
    uint32_t packet_number;
    uint32_t uptime_ms;
    int8_t tx_power_dbm;
    uint8_t rf_profile;
    uint16_t status_flags;
    int16_t local_rssi_dbm_x100;
    int16_t local_snr_db_x100;
    int16_t remote_rssi_dbm_x100;
    int16_t remote_snr_db_x100;
    uint32_t response_time;
    uint16_t payload_crc16;
} range_packet_t;

_Static_assert(sizeof(range_packet_t) <= 255u, "range_packet_t must fit in LoRa payload");

typedef struct __attribute__((packed)) {
    uint8_t version;
    uint8_t msg_type;
    uint8_t role;
    uint8_t radio;
    uint32_t test_id;
    uint32_t window_id;
    uint8_t sweep_mode;
    uint8_t config_id;
    uint8_t direction;
    uint8_t reserved;
} range_ctrl_header_t;

typedef struct __attribute__((packed)) {
    range_ctrl_header_t h;
    uint32_t uptime_ms;
    uint32_t last_window_seen;
    uint16_t status_flags;
} range_heartbeat_t;

typedef struct __attribute__((packed)) {
    range_ctrl_header_t h;
    int8_t tx_power_dbm;
    uint8_t sf;
    uint16_t bw_khz;
    uint8_t cr;
    uint16_t interval_ms;
    uint16_t warmup_ms;
    uint16_t window_ms;
    uint16_t expected_packets;
    uint16_t announce_seq;
} range_config_announce_t;

typedef struct __attribute__((packed)) {
    range_ctrl_header_t h;
    uint32_t seq;
    uint32_t tx_uptime_ms;
    int8_t tx_power_dbm;
    uint8_t rf_profile;
} range_window_data_t;

typedef struct __attribute__((packed)) {
    range_ctrl_header_t h;
    int8_t tx_power_dbm;
    uint8_t sf;
    uint16_t bw_khz;
    uint8_t cr;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t crc_error_count;
    uint16_t prr_x100;
    uint16_t drop_x100;
    uint32_t goodput_bps;
    int16_t rssi_mean_x100;
    int16_t snr_mean_x100;
} range_window_summary_t;

_Static_assert(sizeof(range_ctrl_header_t) <= 255u, "range_ctrl_header_t must fit in LoRa payload");
_Static_assert(sizeof(range_heartbeat_t) <= 255u, "range_heartbeat_t must fit in LoRa payload");
_Static_assert(sizeof(range_config_announce_t) <= 255u, "range_config_announce_t must fit in LoRa payload");
_Static_assert(sizeof(range_window_data_t) <= 255u, "range_window_data_t must fit in LoRa payload");
_Static_assert(sizeof(range_window_summary_t) <= 255u, "range_window_summary_t must fit in LoRa payload");

#endif
