#ifndef BALLOON_PACKET_H
#define BALLOON_PACKET_H

#include <stdint.h>

#define BALLOON_MAGIC       0x424Cu  /* "BL" */
#define CALLSIGN            "KC3WNY"
#define CALLSIGN_LEN        6u

/* ---- message types ---- */

typedef enum {
    BALLOON_MSG_BEACON     = 1u,
    BALLOON_MSG_CMD        = 2u,
    BALLOON_MSG_CMD_ACK    = 3u,
    BALLOON_MSG_BULK       = 4u
} balloon_msg_type_t;

/* ---- command IDs ---- */

typedef enum {
    BALLOON_CMD_PING              = 0u,
    BALLOON_CMD_SET_SBAND_PROFILE = 1u,
    BALLOON_CMD_START_IMAGE       = 2u,
    BALLOON_CMD_START_BULK        = 3u,
    BALLOON_CMD_STOP              = 4u,
    BALLOON_CMD_POWER_SWEEP       = 5u
} balloon_cmd_id_t;

/* ---- flight states ---- */

typedef enum {
    BALLOON_STATE_BEACON       = 0u,
    BALLOON_STATE_IMAGE        = 1u,
    BALLOON_STATE_BULK         = 2u,
    BALLOON_STATE_POWER_SWEEP  = 3u
} balloon_state_t;

/* ---- common 4-byte header ---- */

typedef struct __attribute__((packed)) {
    uint16_t magic;     /* 0x424C */
    uint8_t  msg_type;  /* balloon_msg_type_t */
    uint8_t  seq;       /* rolling 8-bit sequence counter */
} balloon_header_t;

_Static_assert(sizeof(balloon_header_t) == 4u, "balloon_header_t must be 4 bytes");

/* ---- beacon (24 bytes) ---- */

typedef struct __attribute__((packed)) {
    balloon_header_t h;
    uint32_t uptime_ms;
    uint32_t tx_count;
    uint32_t rx_count;
    int16_t  last_cmd_rssi_x100;
    int16_t  last_cmd_snr_x100;
    int8_t   tx_power_dbm;
    uint8_t  sband_profile;
    uint8_t  state;             /* balloon_state_t */
    uint8_t  flags;             /* bit 0: image_loaded, bit 1: bulk_active */
} balloon_beacon_t;

_Static_assert(sizeof(balloon_beacon_t) == 24u, "balloon_beacon_t must be 24 bytes");

#define BALLOON_FLAG_IMAGE_LOADED  (1u << 0)
#define BALLOON_FLAG_BULK_ACTIVE   (1u << 1)

/* ---- command (8 bytes) — ground→balloon on UHF only ---- */

typedef struct __attribute__((packed)) {
    balloon_header_t h;
    uint8_t  cmd_id;    /* balloon_cmd_id_t */
    uint8_t  param1;
    uint8_t  param2;
    uint8_t  reserved;
} balloon_cmd_t;

_Static_assert(sizeof(balloon_cmd_t) == 8u, "balloon_cmd_t must be 8 bytes");

/* ---- command ACK (12 bytes) — balloon→ground on UHF ---- */

typedef struct __attribute__((packed)) {
    balloon_header_t h;
    uint8_t  cmd_id;            /* echoed command */
    uint8_t  ack_seq;           /* echoed seq from command */
    uint8_t  status;            /* 0 = OK */
    uint8_t  reserved;
    int16_t  cmd_rssi_x100;     /* RSSI at which cmd was received */
    int16_t  cmd_snr_x100;      /* SNR at which cmd was received */
} balloon_cmd_ack_t;

_Static_assert(sizeof(balloon_cmd_ack_t) == 12u, "balloon_cmd_ack_t must be 12 bytes");

/* ---- bulk data packet (variable, up to 253 bytes) ---- */

#define BALLOON_BULK_HEADER_SIZE  8u
#define BALLOON_BULK_MAX_PAYLOAD  (253u - BALLOON_BULK_HEADER_SIZE)

typedef struct __attribute__((packed)) {
    balloon_header_t h;
    uint16_t seed;
    uint8_t  data_len;
    uint8_t  flags;         /* bit 0: 0=PRBS, 1=text */
    uint8_t  data[BALLOON_BULK_MAX_PAYLOAD];
} balloon_bulk_t;

_Static_assert(sizeof(balloon_bulk_t) <= 253u, "balloon_bulk_t exceeds SX1280 payload limit");

#define BALLOON_BULK_FLAG_TEXT  (1u << 0)

/* ---- image packets reuse image_packet.h directly ---- */

#endif /* BALLOON_PACKET_H */
