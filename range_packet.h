#ifndef RANGE_PACKET_H
#define RANGE_PACKET_H

#include <limits.h>
#include <stdint.h>

#define RANGE_PACKET_VERSION 1u

typedef enum {
    RANGE_MSG_TRANSMIT = 1u,
    RANGE_MSG_RECEIVE = 2u,
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

#endif
