#ifndef RADIO_RFM98PW_H
#define RADIO_RFM98PW_H

#include <stdint.h>

#include "radio/radio_types.h"

radio_status_t rfm98pw_init(void);
radio_status_t rfm98pw_set_profile(uint8_t rf_profile);
radio_status_t rfm98pw_set_tx_power_dbm(int8_t dbm);
radio_status_t rfm98pw_start_tx(const uint8_t *payload, uint8_t length, uint32_t timeout_ms);
radio_status_t rfm98pw_start_rx(uint32_t timeout_ms);
radio_status_t rfm98pw_poll_event(radio_event_t *event);
radio_status_t rfm98pw_read_rx(radio_rx_frame_t *out_frame);
radio_status_t rfm98pw_abort(void);

#endif
