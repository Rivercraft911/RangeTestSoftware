#ifndef BALLOON_PACKET_H
#define BALLOON_PACKET_H

#include <stdint.h>

#define BALLOON_MAGIC           0x424Cu     /* "BL" */
#define BALLOON_CALLSIGN        "KC3WNY"
#define BALLOON_CALLSIGN_LEN    6u
#define UHF_MAX_PAYLOAD         249u        /* 255 - CALLSIGN_LEN */
#define SBAND_MAX_PAYLOAD       253u

/* ---- Message types ---- */
typedef enum {
    BALLOON_MSG_BEACON      = 1u,
    BALLOON_MSG_CMD         = 2u,
    BALLOON_MSG_CMD_ACK     = 3u,
    BALLOON_MSG_BULK        = 4u
} balloon_msg_type_t;

/* ---- Command IDs ---- */
typedef enum {
    BALLOON_CMD_PING            = 0u,
    BALLOON_CMD_SET_SBAND_PROF  = 1u,
    BALLOON_CMD_START_IMAGE     = 2u,
    BALLOON_CMD_START_BULK      = 3u,
    BALLOON_CMD_STOP            = 4u,
    BALLOON_CMD_POWER_SWEEP     = 5u
} balloon_cmd_id_t;

/* ---- Flight states ---- */
typedef enum {
    BALLOON_STATE_BEACON        = 0u,
    BALLOON_STATE_IMAGE         = 1u,
    BALLOON_STATE_BULK          = 2u,
    BALLOON_STATE_POWER_SWEEP   = 3u
} balloon_state_t;

/* ---- Beacon flags ---- */
#define BALLOON_FLAG_SBAND_OK       (1u << 0)
#define BALLOON_FLAG_UHF_OK         (1u << 1)
#define BALLOON_FLAG_IMAGE_LOADED   (1u << 2)

/* ---- Band IDs (matches range_radio_t) ---- */
#define BALLOON_BAND_UHF    1u
#define BALLOON_BAND_SBAND  2u

/* ---- Common header (4 bytes) ---- */
typedef struct __attribute__((packed)) {
    uint16_t magic;         /* BALLOON_MAGIC */
    uint8_t  msg_type;      /* balloon_msg_type_t */
    uint8_t  seq;           /* rolling sequence number */
} balloon_header_t;

_Static_assert(sizeof(balloon_header_t) == 4u, "balloon_header_t size");

/* ---- Beacon: flight -> ground, UHF (24 bytes) ---- */
typedef struct __attribute__((packed)) {
    balloon_header_t hdr;
    uint32_t uptime_ms;
    uint32_t tx_count;
    uint32_t rx_count;
    int16_t  last_cmd_rssi_x100;
    int16_t  last_cmd_snr_x100;
    int8_t   tx_power_dbm;
    uint8_t  sband_profile;
    uint8_t  state;             /* balloon_state_t */
    uint8_t  flags;
} balloon_beacon_t;

_Static_assert(sizeof(balloon_beacon_t) == 24u, "balloon_beacon_t size");
_Static_assert(sizeof(balloon_beacon_t) <= UHF_MAX_PAYLOAD, "beacon exceeds UHF limit");

/* ---- Command: ground -> flight, UHF (6 bytes) ---- */
typedef struct __attribute__((packed)) {
    balloon_header_t hdr;
    uint8_t  cmd_id;        /* balloon_cmd_id_t */
    uint8_t  param;         /* command-specific parameter */
} balloon_cmd_t;

_Static_assert(sizeof(balloon_cmd_t) == 6u, "balloon_cmd_t size");
_Static_assert(sizeof(balloon_cmd_t) <= UHF_MAX_PAYLOAD, "cmd exceeds UHF limit");

/* ---- Command ACK: flight -> ground, UHF (10 bytes) ---- */
typedef struct __attribute__((packed)) {
    balloon_header_t hdr;
    uint8_t  cmd_id;        /* echoes received command */
    uint8_t  result;        /* 0 = ok, nonzero = error */
    int16_t  cmd_rssi_x100; /* RSSI at which we received the command */
    int16_t  cmd_snr_x100;  /* SNR at which we received the command */
} balloon_cmd_ack_t;

_Static_assert(sizeof(balloon_cmd_ack_t) == 10u, "balloon_cmd_ack_t size");
_Static_assert(sizeof(balloon_cmd_ack_t) <= UHF_MAX_PAYLOAD, "cmd_ack exceeds UHF limit");

/* ---- Bulk data: flight -> ground, S-Band or UHF ---- */
#define BALLOON_BULK_HDR_SIZE   10u
#define BALLOON_BULK_DATA_MAX   (SBAND_MAX_PAYLOAD - BALLOON_BULK_HDR_SIZE)

typedef struct __attribute__((packed)) {
    balloon_header_t hdr;
    uint16_t pkt_num;       /* 0-based packet index */
    uint16_t total_pkts;    /* total packets in transfer */
    uint8_t  data_len;      /* bytes used in data[] (0 = end marker) */
    uint8_t  band;          /* BALLOON_BAND_UHF or BALLOON_BAND_SBAND */
    uint8_t  data[BALLOON_BULK_DATA_MAX];
} balloon_bulk_t;

_Static_assert(sizeof(balloon_bulk_t) == SBAND_MAX_PAYLOAD, "balloon_bulk_t size");
_Static_assert(BALLOON_BULK_HDR_SIZE == 10u, "bulk header math");

/* ---- UHF callsign helper ---- */
static inline uint8_t uhf_append_callsign(uint8_t *buf, uint8_t len) {
    static const char cs[] = BALLOON_CALLSIGN;
    for (uint8_t i = 0; i < BALLOON_CALLSIGN_LEN; i++)
        buf[len + i] = (uint8_t)cs[i];
    return len + BALLOON_CALLSIGN_LEN;
}

#endif /* BALLOON_PACKET_H */
