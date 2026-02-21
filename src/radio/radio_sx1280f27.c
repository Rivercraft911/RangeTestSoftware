#include "radio/radio_sx1280f27.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "hal/board_pins.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "pico/time.h"

enum {
    SX1280_CMD_GET_STATUS = 0xC0,
    SX1280_CMD_WRITE_REGISTER = 0x18,
    SX1280_CMD_READ_REGISTER = 0x19,
    SX1280_CMD_WRITE_BUFFER = 0x1A,
    SX1280_CMD_READ_BUFFER = 0x1B,
    SX1280_CMD_SET_STANDBY = 0x80,
    SX1280_CMD_SET_TX = 0x83,
    SX1280_CMD_SET_RX = 0x82,
    SX1280_CMD_SET_PACKET_TYPE = 0x8A,
    SX1280_CMD_SET_RF_FREQUENCY = 0x86,
    SX1280_CMD_SET_TX_PARAMS = 0x8E,
    SX1280_CMD_SET_BUFFER_BASE_ADDRESS = 0x8F,
    SX1280_CMD_SET_MODULATION_PARAMS = 0x8B,
    SX1280_CMD_SET_PACKET_PARAMS = 0x8C,
    SX1280_CMD_GET_RX_BUFFER_STATUS = 0x17,
    SX1280_CMD_GET_PACKET_STATUS = 0x1D,
    SX1280_CMD_SET_DIO_IRQ_PARAMS = 0x8D,
    SX1280_CMD_GET_IRQ_STATUS = 0x15,
    SX1280_CMD_CLEAR_IRQ_STATUS = 0x97
};

enum {
    SX1280_STANDBY_RC = 0x00,
    SX1280_PACKET_TYPE_LORA = 0x01,
    SX1280_PERIOD_BASE_15_625_US = 0x00,
    SX1280_PERIOD_BASE_1_MS = 0x02,
    SX1280_LORA_SF_5 = 0x50,
    SX1280_LORA_BW_203_125 = 0x34,
    SX1280_LORA_CR_4_5 = 0x01,
    SX1280_LORA_HEADER_EXPLICIT = 0x00,
    SX1280_LORA_CRC_ON = 0x20,
    SX1280_LORA_IQ_STANDARD = 0x40,
    SX1280_PA_RAMP_10_US = 0x80
};

enum {
    SX1280_IRQ_RX_TX_TIMEOUT = 0x4000,
    SX1280_IRQ_CRC_ERROR = 0x0040,
    SX1280_IRQ_RX_DONE = 0x0002,
    SX1280_IRQ_TX_DONE = 0x0001,
    SX1280_IRQ_ALL = 0xFFFF
};

typedef enum {
    SX1280_STATE_IDLE = 0,
    SX1280_STATE_TX_WAIT,
    SX1280_STATE_RX_WAIT,
    SX1280_STATE_RX_READY,
    SX1280_STATE_ERROR
} sx1280_state_t;

typedef struct {
    bool initialized;
    bool rx_valid;
    sx1280_state_t state;
    uint8_t rf_profile;
    int8_t tx_power_dbm;
    uint64_t deadline_ms;
    radio_rx_frame_t rx_frame;
} sx1280_ctx_t;

enum {
    SX1280_REG_VERSION_STRING = 0x01F0u
};

#ifndef SX1280F27_DEBUG
#define SX1280F27_DEBUG 1
#endif

#if SX1280F27_DEBUG
#define SX1280_LOG(fmt, ...) printf("[sx1280] " fmt "\n", ##__VA_ARGS__)
#else
#define SX1280_LOG(fmt, ...) ((void)0)
#endif

static sx1280_ctx_t g_sx1280 = {
    .initialized = false,
    .rx_valid = false,
    .state = SX1280_STATE_IDLE,
    .rf_profile = 1u,
    .tx_power_dbm = 10
};

#if SX1280F27_DEBUG
static uint64_t g_last_poll_diag_ms = 0u;
#endif

static uint8_t sx1280_status_mode(uint8_t status) {
    return (uint8_t)((status >> 5) & 0x07u);
}

static uint8_t sx1280_status_cmd(uint8_t status) {
    return (uint8_t)((status >> 2) & 0x07u);
}

static bool sx1280_status_has_cmd_error(uint8_t status) {
    uint8_t cmd = sx1280_status_cmd(status);
    return (cmd == 0x03u) || (cmd == 0x04u) || (cmd == 0x05u);
}

static const char *sx1280_cmd_status_str(uint8_t status) {
    switch (sx1280_status_cmd(status)) {
        case 0x01:
            return "CMD_OK";
        case 0x02:
            return "DATA_AVAILABLE";
        case 0x03:
            return "CMD_TIMEOUT";
        case 0x04:
            return "CMD_PROCESSING_ERROR";
        case 0x05:
            return "CMD_EXEC_FAILURE";
        case 0x06:
            return "CMD_TX_DONE";
        default:
            return "CMD_RESERVED";
    }
}

static radio_status_t sx1280_get_status_byte(uint8_t *status);

static uint64_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

static bool deadline_expired(uint64_t deadline_ms) {
    return (deadline_ms != UINT64_MAX) && (now_ms() >= deadline_ms);
}

static void set_deadline(uint32_t timeout_ms) {
    if (timeout_ms == 0u) {
        g_sx1280.deadline_ms = UINT64_MAX;
    } else {
        g_sx1280.deadline_ms = now_ms() + timeout_ms;
    }
}

static bool sx1280_wait_while_busy(uint32_t timeout_ms) {
    uint64_t start = time_us_64();
    while (gpio_get(PIN_SBAND_BUSY)) {
        if ((time_us_64() - start) > ((uint64_t)timeout_ms * 1000u)) {
            return false;
        }
        tight_loop_contents();
    }
    return true;
}

static radio_status_t sx1280_write_cmd(uint8_t cmd, const uint8_t *data, size_t len) {
    if (!sx1280_wait_while_busy(50u)) {
        SX1280_LOG("write_cmd(0x%02X) timeout waiting for BUSY low (pre)", cmd);
        return RADIO_STATUS_TIMEOUT;
    }

    gpio_put(PIN_SBAND_CS, 0);
    int written = spi_write_blocking(SBAND_SPI_PORT, &cmd, 1);
    if (written != 1) {
        gpio_put(PIN_SBAND_CS, 1);
        SX1280_LOG("write_cmd(0x%02X) SPI write failed on opcode", cmd);
        return RADIO_STATUS_SPI_ERROR;
    }
    if ((data != NULL) && (len > 0u)) {
        written = spi_write_blocking(SBAND_SPI_PORT, data, len);
        if (written != (int)len) {
            gpio_put(PIN_SBAND_CS, 1);
            SX1280_LOG("write_cmd(0x%02X) SPI write failed on payload len=%u", cmd, (unsigned)len);
            return RADIO_STATUS_SPI_ERROR;
        }
    }
    gpio_put(PIN_SBAND_CS, 1);

    if (!sx1280_wait_while_busy(50u)) {
        SX1280_LOG("write_cmd(0x%02X) timeout waiting for BUSY low (post)", cmd);
        return RADIO_STATUS_TIMEOUT;
    }
#if SX1280F27_DEBUG
    uint8_t status = 0u;
    if (sx1280_get_status_byte(&status) == RADIO_STATUS_OK) {
        if (sx1280_status_has_cmd_error(status)) {
            SX1280_LOG("write_cmd(0x%02X) chip status error: 0x%02X (%s)",
                       cmd, status, sx1280_cmd_status_str(status));
        }
    }
#endif
    return RADIO_STATUS_OK;
}

static radio_status_t sx1280_read_cmd(uint8_t cmd, uint8_t *out, size_t out_len) {
    if ((out == NULL) && (out_len > 0u)) {
        return RADIO_STATUS_INVALID_ARG;
    }
    if (!sx1280_wait_while_busy(50u)) {
        SX1280_LOG("read_cmd(0x%02X) timeout waiting for BUSY low (pre)", cmd);
        return RADIO_STATUS_TIMEOUT;
    }

    gpio_put(PIN_SBAND_CS, 0);
    const uint8_t header[2] = {cmd, 0x00};
    int written = spi_write_blocking(SBAND_SPI_PORT, header, sizeof(header));
    if (written != (int)sizeof(header)) {
        gpio_put(PIN_SBAND_CS, 1);
        SX1280_LOG("read_cmd(0x%02X) SPI write failed on header", cmd);
        return RADIO_STATUS_SPI_ERROR;
    }
    if (out_len > 0u) {
        int read = spi_read_blocking(SBAND_SPI_PORT, 0x00, out, out_len);
        if (read != (int)out_len) {
            gpio_put(PIN_SBAND_CS, 1);
            SX1280_LOG("read_cmd(0x%02X) SPI read failed len=%u", cmd, (unsigned)out_len);
            return RADIO_STATUS_SPI_ERROR;
        }
    }
    gpio_put(PIN_SBAND_CS, 1);

    if (!sx1280_wait_while_busy(50u)) {
        SX1280_LOG("read_cmd(0x%02X) timeout waiting for BUSY low (post)", cmd);
        return RADIO_STATUS_TIMEOUT;
    }
#if SX1280F27_DEBUG
    uint8_t status = 0u;
    if (sx1280_get_status_byte(&status) == RADIO_STATUS_OK) {
        if (sx1280_status_has_cmd_error(status)) {
            SX1280_LOG("read_cmd(0x%02X) chip status error: 0x%02X (%s)",
                       cmd, status, sx1280_cmd_status_str(status));
        }
    }
#endif
    return RADIO_STATUS_OK;
}

static radio_status_t sx1280_read_buffer(uint8_t offset, uint8_t *out, size_t out_len) {
    if ((out == NULL) && (out_len > 0u)) {
        return RADIO_STATUS_INVALID_ARG;
    }
    if (!sx1280_wait_while_busy(50u)) {
        SX1280_LOG("read_buffer(offset=%u) timeout waiting for BUSY low (pre)", (unsigned)offset);
        return RADIO_STATUS_TIMEOUT;
    }

    const uint8_t header[3] = {SX1280_CMD_READ_BUFFER, offset, 0x00};
    gpio_put(PIN_SBAND_CS, 0);
    int written = spi_write_blocking(SBAND_SPI_PORT, header, sizeof(header));
    if (written != (int)sizeof(header)) {
        gpio_put(PIN_SBAND_CS, 1);
        SX1280_LOG("read_buffer(offset=%u) SPI write failed on header", (unsigned)offset);
        return RADIO_STATUS_SPI_ERROR;
    }
    if (out_len > 0u) {
        int read = spi_read_blocking(SBAND_SPI_PORT, 0x00, out, out_len);
        if (read != (int)out_len) {
            gpio_put(PIN_SBAND_CS, 1);
            SX1280_LOG("read_buffer(offset=%u) SPI read failed len=%u", (unsigned)offset, (unsigned)out_len);
            return RADIO_STATUS_SPI_ERROR;
        }
    }
    gpio_put(PIN_SBAND_CS, 1);

    if (!sx1280_wait_while_busy(50u)) {
        SX1280_LOG("read_buffer(offset=%u) timeout waiting for BUSY low (post)", (unsigned)offset);
        return RADIO_STATUS_TIMEOUT;
    }
    return RADIO_STATUS_OK;
}

static radio_status_t sx1280_set_irq_params(uint16_t irq_mask, uint16_t dio1_mask) {
    const uint8_t data[] = {
        (uint8_t)(irq_mask >> 8), (uint8_t)(irq_mask & 0xFF),
        (uint8_t)(dio1_mask >> 8), (uint8_t)(dio1_mask & 0xFF),
        0x00, 0x00, 0x00, 0x00
    };
    return sx1280_write_cmd(SX1280_CMD_SET_DIO_IRQ_PARAMS, data, sizeof(data));
}

static radio_status_t sx1280_clear_irq(uint16_t irq_mask) {
    const uint8_t data[] = {
        (uint8_t)(irq_mask >> 8),
        (uint8_t)(irq_mask & 0xFF)
    };
    return sx1280_write_cmd(SX1280_CMD_CLEAR_IRQ_STATUS, data, sizeof(data));
}

static radio_status_t sx1280_get_irq(uint16_t *irq) {
    if (irq == NULL) {
        return RADIO_STATUS_INVALID_ARG;
    }
    uint8_t data[2] = {0u, 0u};
    radio_status_t st = sx1280_read_cmd(SX1280_CMD_GET_IRQ_STATUS, data, sizeof(data));
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    *irq = ((uint16_t)data[0] << 8) | data[1];
    return RADIO_STATUS_OK;
}

static radio_status_t sx1280_get_status_byte_one_byte(uint8_t *status) {
    if (status == NULL) {
        return RADIO_STATUS_INVALID_ARG;
    }
    if (!sx1280_wait_while_busy(50u)) {
        SX1280_LOG("get_status(1-byte) timeout waiting for BUSY low (pre)");
        return RADIO_STATUS_TIMEOUT;
    }

    uint8_t tx = SX1280_CMD_GET_STATUS;
    uint8_t rx = 0u;
    gpio_put(PIN_SBAND_CS, 0);
    int read = spi_write_read_blocking(SBAND_SPI_PORT, &tx, &rx, 1u);
    gpio_put(PIN_SBAND_CS, 1);
    if (read != 1) {
        SX1280_LOG("get_status(1-byte) SPI transfer failed");
        return RADIO_STATUS_SPI_ERROR;
    }

    if (!sx1280_wait_while_busy(50u)) {
        SX1280_LOG("get_status(1-byte) timeout waiting for BUSY low (post)");
        return RADIO_STATUS_TIMEOUT;
    }

    *status = rx;
    return RADIO_STATUS_OK;
}

static radio_status_t sx1280_get_status_byte_two_byte(uint8_t *status) {
    if (status == NULL) {
        return RADIO_STATUS_INVALID_ARG;
    }
    if (!sx1280_wait_while_busy(50u)) {
        SX1280_LOG("get_status(2-byte) timeout waiting for BUSY low (pre)");
        return RADIO_STATUS_TIMEOUT;
    }

    const uint8_t tx[2] = {SX1280_CMD_GET_STATUS, 0x00u};
    uint8_t rx[2] = {0u, 0u};
    gpio_put(PIN_SBAND_CS, 0);
    int read = spi_write_read_blocking(SBAND_SPI_PORT, tx, rx, 2u);
    gpio_put(PIN_SBAND_CS, 1);
    if (read != 2) {
        SX1280_LOG("get_status(2-byte) SPI transfer failed");
        return RADIO_STATUS_SPI_ERROR;
    }

    if (!sx1280_wait_while_busy(50u)) {
        SX1280_LOG("get_status(2-byte) timeout waiting for BUSY low (post)");
        return RADIO_STATUS_TIMEOUT;
    }

    *status = rx[1];
    return RADIO_STATUS_OK;
}

static radio_status_t sx1280_get_status_byte(uint8_t *status) {
    uint8_t one = 0u;
    radio_status_t st = sx1280_get_status_byte_one_byte(&one);
    if (st != RADIO_STATUS_OK) {
        return st;
    }

    if ((one == 0x00u) || (one == 0xFFu)) {
        uint8_t two = 0u;
        radio_status_t st2 = sx1280_get_status_byte_two_byte(&two);
        if ((st2 == RADIO_STATUS_OK) && (two != 0x00u) && (two != 0xFFu)) {
            one = two;
        }
    }

    *status = one;
    return RADIO_STATUS_OK;
}

static radio_status_t sx1280_read_register(uint16_t addr, uint8_t *out, size_t out_len) {
    if ((out == NULL) && (out_len > 0u)) {
        return RADIO_STATUS_INVALID_ARG;
    }
    if (!sx1280_wait_while_busy(50u)) {
        SX1280_LOG("read_register(0x%04X) timeout waiting for BUSY low (pre)", (unsigned)addr);
        return RADIO_STATUS_TIMEOUT;
    }

    const uint8_t header[4] = {
        SX1280_CMD_READ_REGISTER,
        (uint8_t)((addr >> 8) & 0xFFu),
        (uint8_t)(addr & 0xFFu),
        0x00u
    };
    gpio_put(PIN_SBAND_CS, 0);
    int written = spi_write_blocking(SBAND_SPI_PORT, header, sizeof(header));
    if (written != (int)sizeof(header)) {
        gpio_put(PIN_SBAND_CS, 1);
        SX1280_LOG("read_register(0x%04X) SPI write failed on header", (unsigned)addr);
        return RADIO_STATUS_SPI_ERROR;
    }
    if (out_len > 0u) {
        int read = spi_read_blocking(SBAND_SPI_PORT, 0x00, out, out_len);
        if (read != (int)out_len) {
            gpio_put(PIN_SBAND_CS, 1);
            SX1280_LOG("read_register(0x%04X) SPI read failed len=%u", (unsigned)addr, (unsigned)out_len);
            return RADIO_STATUS_SPI_ERROR;
        }
    }
    gpio_put(PIN_SBAND_CS, 1);

    if (!sx1280_wait_while_busy(50u)) {
        SX1280_LOG("read_register(0x%04X) timeout waiting for BUSY low (post)", (unsigned)addr);
        return RADIO_STATUS_TIMEOUT;
    }
    return RADIO_STATUS_OK;
}

static void sx1280_log_status_snapshot(const char *tag) {
#if SX1280F27_DEBUG
    uint8_t status = 0u;
    radio_status_t st = sx1280_get_status_byte(&status);
    if (st != RADIO_STATUS_OK) {
        SX1280_LOG("%s: status read failed st=%d", tag, (int)st);
        return;
    }
    SX1280_LOG("%s: status=0x%02X mode=0x%X cmd=%s busy=%d dio1=%d",
               tag,
               status,
               (unsigned)sx1280_status_mode(status),
               sx1280_cmd_status_str(status),
               gpio_get(PIN_SBAND_BUSY),
               gpio_get(PIN_SBAND_DIO1));
#else
    (void)tag;
#endif
}

static radio_status_t sx1280_set_standby(void) {
    const uint8_t mode = SX1280_STANDBY_RC;
    return sx1280_write_cmd(SX1280_CMD_SET_STANDBY, &mode, 1u);
}

static radio_status_t sx1280_set_frequency(uint32_t freq_hz) {
    // SX128x frequency step is Fxtal / 2^18 where Fxtal is 52 MHz.
    uint32_t frf = (uint32_t)(((uint64_t)freq_hz << 18) / 52000000u);
    const uint8_t data[] = {
        (uint8_t)((frf >> 16) & 0xFFu),
        (uint8_t)((frf >> 8) & 0xFFu),
        (uint8_t)(frf & 0xFFu)
    };
    return sx1280_write_cmd(SX1280_CMD_SET_RF_FREQUENCY, data, sizeof(data));
}

static radio_status_t sx1280_set_modulation_lora_profile_1(void) {
    const uint8_t data[] = {
        SX1280_LORA_SF_5,
        SX1280_LORA_BW_203_125,
        SX1280_LORA_CR_4_5
    };
    return sx1280_write_cmd(SX1280_CMD_SET_MODULATION_PARAMS, data, sizeof(data));
}

static radio_status_t sx1280_set_packet_params_lora(uint8_t payload_len) {
    const uint8_t data[] = {
        12u,                                 // preamble length (recommended for bring-up)
        SX1280_LORA_HEADER_EXPLICIT,         // explicit header
        payload_len,
        SX1280_LORA_CRC_ON,                  // CRC enabled
        SX1280_LORA_IQ_STANDARD,             // standard IQ
        0x00,
        0x00
    };
    return sx1280_write_cmd(SX1280_CMD_SET_PACKET_PARAMS, data, sizeof(data));
}

static radio_status_t sx1280_set_tx_params(int8_t dbm) {
    if (dbm > 13) {
        dbm = 13;
    } else if (dbm < -18) {
        dbm = -18;
    }
    // SX1280 expects power in [0..31], mapping to physical [-18..13] dBm.
    const uint8_t power_code = (uint8_t)(dbm + 18);
    const uint8_t data[] = {power_code, SX1280_PA_RAMP_10_US};
    return sx1280_write_cmd(SX1280_CMD_SET_TX_PARAMS, data, sizeof(data));
}

static radio_status_t sx1280_apply_profile_1(void) {
    const uint8_t packet_type = SX1280_PACKET_TYPE_LORA;
    const uint8_t buffer_base[] = {0x00, 0x00};
    radio_status_t st = sx1280_write_cmd(SX1280_CMD_SET_PACKET_TYPE, &packet_type, 1u);
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = sx1280_write_cmd(SX1280_CMD_SET_BUFFER_BASE_ADDRESS, buffer_base, sizeof(buffer_base));
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = sx1280_set_frequency(2427000000u);
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = sx1280_set_modulation_lora_profile_1();
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = sx1280_set_packet_params_lora(253u);
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = sx1280_set_tx_params(g_sx1280.tx_power_dbm);
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = sx1280_set_irq_params(
        (uint16_t)(SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE | SX1280_IRQ_RX_TX_TIMEOUT | SX1280_IRQ_CRC_ERROR),
        (uint16_t)(SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE | SX1280_IRQ_RX_TX_TIMEOUT | SX1280_IRQ_CRC_ERROR));
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    return sx1280_clear_irq(SX1280_IRQ_ALL);
}

static radio_status_t sx1280_start_tx_or_rx(uint8_t cmd, uint8_t period_base, uint16_t period_base_count) {
    const uint8_t data[] = {
        period_base,
        (uint8_t)(period_base_count >> 8),
        (uint8_t)(period_base_count & 0xFFu)
    };
    return sx1280_write_cmd(cmd, data, sizeof(data));
}

static radio_status_t sx1280_read_rx_frame(void) {
    uint8_t rx_buf_status[2] = {0u, 0u};
    radio_status_t st = sx1280_read_cmd(SX1280_CMD_GET_RX_BUFFER_STATUS, rx_buf_status, sizeof(rx_buf_status));
    if (st != RADIO_STATUS_OK) {
        return st;
    }

    uint8_t payload_len = rx_buf_status[0];
    uint8_t offset = rx_buf_status[1];
    if (payload_len > sizeof(g_sx1280.rx_frame.data)) {
        return RADIO_STATUS_INTERNAL_ERROR;
    }

    st = sx1280_read_buffer(offset, g_sx1280.rx_frame.data, payload_len);
    if (st != RADIO_STATUS_OK) {
        return st;
    }

    uint8_t packet_status[5] = {0u};
    st = sx1280_read_cmd(SX1280_CMD_GET_PACKET_STATUS, packet_status, sizeof(packet_status));
    if (st != RADIO_STATUS_OK) {
        return st;
    }

    g_sx1280.rx_frame.length = payload_len;
    g_sx1280.rx_frame.rssi_dbm_x100 = (int16_t)(-50 * (int16_t)packet_status[0]);
    g_sx1280.rx_frame.snr_db_x100 = (int16_t)((int16_t)(int8_t)packet_status[1] * 25);
    g_sx1280.rx_valid = true;
    return RADIO_STATUS_OK;
}

static void sx1280_set_rf_switch_tx(void) {
    gpio_put(PIN_SBAND_TXEN, 1);
    gpio_put(PIN_SBAND_RXEN, 0);
}

static void sx1280_set_rf_switch_rx(void) {
    gpio_put(PIN_SBAND_TXEN, 0);
    gpio_put(PIN_SBAND_RXEN, 1);
}

static void sx1280_set_rf_switch_idle(void) {
    gpio_put(PIN_SBAND_TXEN, 0);
    gpio_put(PIN_SBAND_RXEN, 0);
}

radio_status_t sx1280f27_init(void) {
    sx1280_set_rf_switch_idle();
    gpio_put(PIN_SBAND_RST, 0);
    sleep_ms(2);
    gpio_put(PIN_SBAND_RST, 1);
    sleep_ms(5);

    if (!sx1280_wait_while_busy(100u)) {
        g_sx1280.state = SX1280_STATE_ERROR;
        return RADIO_STATUS_TIMEOUT;
    }

    radio_status_t st = sx1280_set_standby();
    if (st != RADIO_STATUS_OK) {
        g_sx1280.state = SX1280_STATE_ERROR;
        return st;
    }

    g_sx1280.rf_profile = 1u;
    g_sx1280.rx_valid = false;
    g_sx1280.state = SX1280_STATE_IDLE;
    g_sx1280.initialized = true;
    st = sx1280_apply_profile_1();
    if (st != RADIO_STATUS_OK) {
        g_sx1280.state = SX1280_STATE_ERROR;
        g_sx1280.initialized = false;
        return st;
    }

    uint8_t status = 0u;
    st = sx1280_get_status_byte(&status);
    if (st != RADIO_STATUS_OK) {
        g_sx1280.state = SX1280_STATE_ERROR;
        g_sx1280.initialized = false;
        return st;
    }
    if ((status == 0x00u) || (status == 0xFFu)) {
        g_sx1280.state = SX1280_STATE_ERROR;
        g_sx1280.initialized = false;
        return RADIO_STATUS_NOT_READY;
    }

    if (sx1280_status_has_cmd_error(status)) {
        SX1280_LOG("init: chip reports command error status=0x%02X (%s)",
                   status, sx1280_cmd_status_str(status));
    }

    uint8_t version[16] = {0u};
    st = sx1280_read_register(SX1280_REG_VERSION_STRING, version, sizeof(version));
    if (st != RADIO_STATUS_OK) {
        SX1280_LOG("init: failed to read version string st=%d", (int)st);
        g_sx1280.state = SX1280_STATE_ERROR;
        g_sx1280.initialized = false;
        return st;
    }
    bool all_zero = true;
    bool all_ff = true;
    for (size_t i = 0; i < sizeof(version); ++i) {
        if (version[i] != 0x00u) {
            all_zero = false;
        }
        if (version[i] != 0xFFu) {
            all_ff = false;
        }
    }
    if (all_zero || all_ff) {
        SX1280_LOG("init: invalid version string (all %s), SPI link likely bad",
                   all_zero ? "0x00" : "0xFF");
        g_sx1280.state = SX1280_STATE_ERROR;
        g_sx1280.initialized = false;
        return RADIO_STATUS_NOT_READY;
    }

#if SX1280F27_DEBUG
    char version_text[17];
    for (size_t i = 0; i < sizeof(version); ++i) {
        uint8_t c = version[i];
        version_text[i] = ((c >= 32u) && (c <= 126u)) ? (char)c : '.';
    }
    version_text[16] = '\0';
    SX1280_LOG("init: version='%s'", version_text);
#endif
    SX1280_LOG("init: status=0x%02X mode=0x%X cmd=%s",
               status,
               (unsigned)sx1280_status_mode(status),
               sx1280_cmd_status_str(status));
    sx1280_log_status_snapshot("after init");

    return RADIO_STATUS_OK;
}

radio_status_t sx1280f27_set_profile(uint8_t rf_profile) {
    if (!g_sx1280.initialized) {
        return RADIO_STATUS_NOT_READY;
    }
    if (rf_profile != 1u) {
        return RADIO_STATUS_INVALID_ARG;
    }
    if ((g_sx1280.state == SX1280_STATE_TX_WAIT) || (g_sx1280.state == SX1280_STATE_RX_WAIT)) {
        return RADIO_STATUS_BUSY;
    }
    g_sx1280.rf_profile = rf_profile;
    return sx1280_apply_profile_1();
}

radio_status_t sx1280f27_set_tx_power_dbm(int8_t dbm) {
    if (!g_sx1280.initialized) {
        return RADIO_STATUS_NOT_READY;
    }
    if ((g_sx1280.state == SX1280_STATE_TX_WAIT) || (g_sx1280.state == SX1280_STATE_RX_WAIT)) {
        return RADIO_STATUS_BUSY;
    }
    g_sx1280.tx_power_dbm = dbm;
    return sx1280_set_tx_params(dbm);
}

radio_status_t sx1280f27_start_tx(const uint8_t *payload, uint8_t length, uint32_t timeout_ms) {
    if (!g_sx1280.initialized) {
        return RADIO_STATUS_NOT_READY;
    }
    if ((payload == NULL) || (length == 0u)) {
        return RADIO_STATUS_INVALID_ARG;
    }
    if (g_sx1280.state != SX1280_STATE_IDLE) {
        return RADIO_STATUS_BUSY;
    }

    radio_status_t st = sx1280_set_standby();
    if (st != RADIO_STATUS_OK) {
        g_sx1280.state = SX1280_STATE_ERROR;
        return st;
    }

    st = sx1280_set_packet_params_lora(length);
    if (st != RADIO_STATUS_OK) {
        g_sx1280.state = SX1280_STATE_ERROR;
        return st;
    }

    uint8_t tx_data[1u + 255u];
    tx_data[0] = 0x00;
    memcpy(&tx_data[1], payload, length);
    st = sx1280_write_cmd(SX1280_CMD_WRITE_BUFFER, tx_data, (size_t)length + 1u);
    if (st != RADIO_STATUS_OK) {
        g_sx1280.state = SX1280_STATE_ERROR;
        return st;
    }

    st = sx1280_set_irq_params((uint16_t)(SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_TX_TIMEOUT), (uint16_t)(SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_TX_TIMEOUT));
    if (st != RADIO_STATUS_OK) {
        g_sx1280.state = SX1280_STATE_ERROR;
        return st;
    }
    st = sx1280_clear_irq(SX1280_IRQ_ALL);
    if (st != RADIO_STATUS_OK) {
        g_sx1280.state = SX1280_STATE_ERROR;
        return st;
    }

    sx1280_set_rf_switch_tx();
    st = sx1280_start_tx_or_rx(SX1280_CMD_SET_TX, SX1280_PERIOD_BASE_15_625_US, 0x0000u);
    if (st != RADIO_STATUS_OK) {
        g_sx1280.state = SX1280_STATE_ERROR;
        sx1280_set_rf_switch_idle();
        return st;
    }

    set_deadline(timeout_ms);
    g_sx1280.state = SX1280_STATE_TX_WAIT;
    sx1280_log_status_snapshot("after start_tx");
    SX1280_LOG("start_tx: len=%u timeout_ms=%u", (unsigned)length, (unsigned)timeout_ms);
    return RADIO_STATUS_OK;
}

radio_status_t sx1280f27_start_rx(uint32_t timeout_ms) {
    if (!g_sx1280.initialized) {
        return RADIO_STATUS_NOT_READY;
    }
    if (g_sx1280.state != SX1280_STATE_IDLE) {
        return RADIO_STATUS_BUSY;
    }

    radio_status_t st = sx1280_set_standby();
    if (st != RADIO_STATUS_OK) {
        g_sx1280.state = SX1280_STATE_ERROR;
        return st;
    }

    g_sx1280.rx_valid = false;
    st = sx1280_set_packet_params_lora(253u);
    if (st != RADIO_STATUS_OK) {
        g_sx1280.state = SX1280_STATE_ERROR;
        return st;
    }
    st = sx1280_set_irq_params(
        (uint16_t)(SX1280_IRQ_RX_DONE | SX1280_IRQ_RX_TX_TIMEOUT | SX1280_IRQ_CRC_ERROR),
        (uint16_t)(SX1280_IRQ_RX_DONE | SX1280_IRQ_RX_TX_TIMEOUT | SX1280_IRQ_CRC_ERROR));
    if (st != RADIO_STATUS_OK) {
        g_sx1280.state = SX1280_STATE_ERROR;
        return st;
    }
    st = sx1280_clear_irq(SX1280_IRQ_ALL);
    if (st != RADIO_STATUS_OK) {
        g_sx1280.state = SX1280_STATE_ERROR;
        return st;
    }

    sx1280_set_rf_switch_rx();
    st = sx1280_start_tx_or_rx(SX1280_CMD_SET_RX, SX1280_PERIOD_BASE_15_625_US, 0xFFFFu);
    if (st != RADIO_STATUS_OK) {
        g_sx1280.state = SX1280_STATE_ERROR;
        sx1280_set_rf_switch_idle();
        return st;
    }

    set_deadline(timeout_ms);
    g_sx1280.state = SX1280_STATE_RX_WAIT;
    sx1280_log_status_snapshot("after start_rx");
    SX1280_LOG("start_rx: timeout_ms=%u", (unsigned)timeout_ms);
    return RADIO_STATUS_OK;
}

radio_status_t sx1280f27_poll_event(radio_event_t *event) {
    if (event == NULL) {
        return RADIO_STATUS_INVALID_ARG;
    }
    *event = RADIO_EVENT_NONE;

    if (!g_sx1280.initialized) {
        return RADIO_STATUS_NOT_READY;
    }

    bool active_wait = (g_sx1280.state == SX1280_STATE_TX_WAIT) || (g_sx1280.state == SX1280_STATE_RX_WAIT);
    if (((g_sx1280.state == SX1280_STATE_TX_WAIT) || (g_sx1280.state == SX1280_STATE_RX_WAIT)) && deadline_expired(g_sx1280.deadline_ms)) {
        sx1280_log_status_snapshot("poll deadline expired");
        (void)sx1280_set_standby();
        (void)sx1280_clear_irq(SX1280_IRQ_ALL);
        sx1280_set_rf_switch_idle();
        g_sx1280.state = SX1280_STATE_IDLE;
        *event = RADIO_EVENT_TIMEOUT;
        return RADIO_STATUS_OK;
    }

    if (!active_wait) {
        return RADIO_STATUS_OK;
    }

    uint16_t irq = 0u;
    radio_status_t st = sx1280_get_irq(&irq);
    if (st != RADIO_STATUS_OK) {
        g_sx1280.state = SX1280_STATE_ERROR;
        *event = RADIO_EVENT_ERROR;
        return st;
    }
    if (irq == 0u) {
#if SX1280F27_DEBUG
        uint64_t now = now_ms();
        if ((now - g_last_poll_diag_ms) >= 250u) {
            uint8_t status = 0u;
            radio_status_t st_status = sx1280_get_status_byte(&status);
            if (st_status == RADIO_STATUS_OK) {
                SX1280_LOG("poll: waiting state=%d irq=0x0000 status=0x%02X(%s) busy=%d dio1=%d",
                           (int)g_sx1280.state,
                           status,
                           sx1280_cmd_status_str(status),
                           gpio_get(PIN_SBAND_BUSY),
                           gpio_get(PIN_SBAND_DIO1));
            }
            g_last_poll_diag_ms = now;
        }
#endif
        if (g_sx1280.state == SX1280_STATE_TX_WAIT) {
            uint8_t status = 0u;
            if (sx1280_get_status_byte(&status) == RADIO_STATUS_OK && sx1280_status_cmd(status) == 0x06u) {
                SX1280_LOG("poll: inferred TX_DONE from status=0x%02X while IRQ was 0", status);
                (void)sx1280_set_standby();
                (void)sx1280_clear_irq(SX1280_IRQ_ALL);
                sx1280_set_rf_switch_idle();
                g_sx1280.state = SX1280_STATE_IDLE;
                *event = RADIO_EVENT_TX_DONE;
                return RADIO_STATUS_OK;
            }
        }
        return RADIO_STATUS_OK;
    }
    SX1280_LOG("poll: irq=0x%04X state=%d busy=%d dio1=%d",
               (unsigned)irq,
               (int)g_sx1280.state,
               gpio_get(PIN_SBAND_BUSY),
               gpio_get(PIN_SBAND_DIO1));
    (void)sx1280_clear_irq(irq);

    if (irq & SX1280_IRQ_CRC_ERROR) {
        (void)sx1280_set_standby();
        sx1280_set_rf_switch_idle();
        g_sx1280.state = SX1280_STATE_IDLE;
        sx1280_log_status_snapshot("poll event CRC_FAIL");
        *event = RADIO_EVENT_CRC_FAIL;
        return RADIO_STATUS_OK;
    }

    if (irq & SX1280_IRQ_RX_TX_TIMEOUT) {
        (void)sx1280_set_standby();
        sx1280_set_rf_switch_idle();
        g_sx1280.state = SX1280_STATE_IDLE;
        sx1280_log_status_snapshot("poll event RADIO_TIMEOUT");
        *event = RADIO_EVENT_TIMEOUT;
        return RADIO_STATUS_OK;
    }

    if (irq & SX1280_IRQ_TX_DONE) {
        (void)sx1280_set_standby();
        sx1280_set_rf_switch_idle();
        g_sx1280.state = SX1280_STATE_IDLE;
        sx1280_log_status_snapshot("poll event TX_DONE");
        *event = RADIO_EVENT_TX_DONE;
        return RADIO_STATUS_OK;
    }

    if (irq & SX1280_IRQ_RX_DONE) {
        st = sx1280_read_rx_frame();
        if (st != RADIO_STATUS_OK) {
            g_sx1280.state = SX1280_STATE_ERROR;
            *event = RADIO_EVENT_ERROR;
            return st;
        }
        (void)sx1280_set_standby();
        sx1280_set_rf_switch_idle();
        g_sx1280.state = SX1280_STATE_RX_READY;
        sx1280_log_status_snapshot("poll event RX_DONE");
        *event = RADIO_EVENT_RX_DONE;
        return RADIO_STATUS_OK;
    }

    return RADIO_STATUS_OK;
}

radio_status_t sx1280f27_read_rx(radio_rx_frame_t *out_frame) {
    if (out_frame == NULL) {
        return RADIO_STATUS_INVALID_ARG;
    }
    if (!g_sx1280.initialized) {
        return RADIO_STATUS_NOT_READY;
    }
    if ((g_sx1280.state != SX1280_STATE_RX_READY) || !g_sx1280.rx_valid) {
        return RADIO_STATUS_NOT_READY;
    }

    *out_frame = g_sx1280.rx_frame;
    g_sx1280.rx_valid = false;
    g_sx1280.state = SX1280_STATE_IDLE;
    return RADIO_STATUS_OK;
}

radio_status_t sx1280f27_abort(void) {
    if (!g_sx1280.initialized) {
        return RADIO_STATUS_NOT_READY;
    }

    radio_status_t st = sx1280_set_standby();
    if (st != RADIO_STATUS_OK) {
        g_sx1280.state = SX1280_STATE_ERROR;
        return st;
    }
    (void)sx1280_clear_irq(SX1280_IRQ_ALL);
    sx1280_set_rf_switch_idle();
    g_sx1280.rx_valid = false;
    g_sx1280.state = SX1280_STATE_IDLE;
    return RADIO_STATUS_OK;
}
