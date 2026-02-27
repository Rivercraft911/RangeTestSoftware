#ifndef IMAGE_PACKET_H
#define IMAGE_PACKET_H

#include <stdint.h>

#define IMAGE_PACKET_MAGIC  0x494Du

typedef enum {
    IMAGE_PKT_START = 0x01u,
    IMAGE_PKT_DATA  = 0x02u,
    IMAGE_PKT_END   = 0x03u
} image_pkt_type_t;

#define IMAGE_HEADER_SIZE   8u
#define IMAGE_DATA_PER_PKT  (253u - IMAGE_HEADER_SIZE)

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t  pkt_type;
    uint16_t pkt_num;
    uint16_t total_pkts;
    uint8_t  data_len;
    uint8_t  data[IMAGE_DATA_PER_PKT];
} image_packet_t;

_Static_assert(sizeof(image_packet_t) <= 253u, "image_packet_t exceeds SX1280 payload limit");

#endif
