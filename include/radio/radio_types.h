#ifndef RADIO_TYPES_H
#define RADIO_TYPES_H

#include <stdint.h>

typedef enum {
    RADIO_STATUS_OK = 0,
    RADIO_STATUS_INVALID_ARG,
    RADIO_STATUS_BUSY,
    RADIO_STATUS_SPI_ERROR,
    RADIO_STATUS_TIMEOUT,
    RADIO_STATUS_CRC_FAIL,
    RADIO_STATUS_NOT_READY,
    RADIO_STATUS_INTERNAL_ERROR
} radio_status_t;

typedef enum {
    RADIO_EVENT_NONE = 0,
    RADIO_EVENT_TX_DONE,
    RADIO_EVENT_RX_DONE,
    RADIO_EVENT_TIMEOUT,
    RADIO_EVENT_CRC_FAIL,
    RADIO_EVENT_ERROR
} radio_event_t;

typedef struct {
    uint8_t data[255];
    uint8_t length;
    int16_t rssi_dbm_x100;
    int16_t snr_db_x100;
} radio_rx_frame_t;

#endif
