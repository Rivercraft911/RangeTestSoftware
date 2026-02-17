#include "radio/radio_rfm98pw.h"

#include <string.h>

#include "hal/board_pins.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "pico/time.h"

enum {
    SX127X_REG_FIFO = 0x00,
    SX127X_REG_OP_MODE = 0x01,
    SX127X_REG_FRF_MSB = 0x06,
    SX127X_REG_FRF_MID = 0x07,
    SX127X_REG_FRF_LSB = 0x08,
    SX127X_REG_PA_CONFIG = 0x09,
    SX127X_REG_FIFO_ADDR_PTR = 0x0D,
    SX127X_REG_FIFO_TX_BASE_ADDR = 0x0E,
    SX127X_REG_FIFO_RX_BASE_ADDR = 0x0F,
    SX127X_REG_FIFO_RX_CURRENT_ADDR = 0x10,
    SX127X_REG_IRQ_FLAGS = 0x12,
    SX127X_REG_RX_NB_BYTES = 0x13,
    SX127X_REG_PKT_SNR_VALUE = 0x19,
    SX127X_REG_PKT_RSSI_VALUE = 0x1A,
    SX127X_REG_MODEM_CONFIG_1 = 0x1D,
    SX127X_REG_MODEM_CONFIG_2 = 0x1E,
    SX127X_REG_SYMB_TIMEOUT_LSB = 0x1F,
    SX127X_REG_PREAMBLE_MSB = 0x20,
    SX127X_REG_PREAMBLE_LSB = 0x21,
    SX127X_REG_PAYLOAD_LENGTH = 0x22,
    SX127X_REG_MODEM_CONFIG_3 = 0x26,
    SX127X_REG_SYNC_WORD = 0x39,
    SX127X_REG_DIO_MAPPING_1 = 0x40,
    SX127X_REG_VERSION = 0x42,
    SX127X_REG_PA_DAC = 0x4D
};

enum {
    SX127X_LONG_RANGE_MODE = 0x80,
    SX127X_MODE_SLEEP = 0x00,
    SX127X_MODE_STDBY = 0x01,
    SX127X_MODE_TX = 0x03,
    SX127X_MODE_RX_SINGLE = 0x06
};

enum {
    SX127X_IRQ_RX_TIMEOUT = 0x80,
    SX127X_IRQ_RX_DONE = 0x40,
    SX127X_IRQ_PAYLOAD_CRC_ERROR = 0x20,
    SX127X_IRQ_TX_DONE = 0x08,
    SX127X_IRQ_ALL = 0xFF
};

typedef enum {
    RFM98_STATE_IDLE = 0,
    RFM98_STATE_TX_WAIT,
    RFM98_STATE_RX_WAIT,
    RFM98_STATE_RX_READY,
    RFM98_STATE_ERROR
} rfm98_state_t;

typedef struct {
    bool initialized;
    bool rx_valid;
    rfm98_state_t state;
    uint8_t rf_profile;
    int8_t tx_power_dbm;
    uint64_t deadline_ms;
    radio_rx_frame_t rx_frame;
} rfm98_ctx_t;

static rfm98_ctx_t g_rfm98 = {
    .initialized = false,
    .rx_valid = false,
    .state = RFM98_STATE_IDLE,
    .rf_profile = 1u,
    .tx_power_dbm = 17
};

static uint64_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

static bool deadline_expired(uint64_t deadline_ms) {
    return (deadline_ms != UINT64_MAX) && (now_ms() >= deadline_ms);
}

static void set_deadline(uint32_t timeout_ms) {
    if (timeout_ms == 0u) {
        g_rfm98.deadline_ms = UINT64_MAX;
    } else {
        g_rfm98.deadline_ms = now_ms() + timeout_ms;
    }
}

static radio_status_t rfm98_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {(uint8_t)(reg | 0x80u), value};
    gpio_put(PIN_UHF_CS, 0);
    int written = spi_write_blocking(UHF_SPI_PORT, buf, sizeof(buf));
    gpio_put(PIN_UHF_CS, 1);
    return (written == (int)sizeof(buf)) ? RADIO_STATUS_OK : RADIO_STATUS_SPI_ERROR;
}

static radio_status_t rfm98_read_reg(uint8_t reg, uint8_t *value) {
    if (value == NULL) {
        return RADIO_STATUS_INVALID_ARG;
    }
    uint8_t tx[2] = {(uint8_t)(reg & 0x7Fu), 0x00u};
    uint8_t rx[2] = {0u, 0u};
    gpio_put(PIN_UHF_CS, 0);
    int read = spi_write_read_blocking(UHF_SPI_PORT, tx, rx, sizeof(tx));
    gpio_put(PIN_UHF_CS, 1);
    if (read != (int)sizeof(tx)) {
        return RADIO_STATUS_SPI_ERROR;
    }
    *value = rx[1];
    return RADIO_STATUS_OK;
}

static radio_status_t rfm98_write_fifo(const uint8_t *data, uint8_t len) {
    if ((data == NULL) && (len > 0u)) {
        return RADIO_STATUS_INVALID_ARG;
    }
    gpio_put(PIN_UHF_CS, 0);
    uint8_t addr = (uint8_t)(SX127X_REG_FIFO | 0x80u);
    int written = spi_write_blocking(UHF_SPI_PORT, &addr, 1);
    if ((written == 1) && (len > 0u)) {
        written = spi_write_blocking(UHF_SPI_PORT, data, len);
        if (written != len) {
            gpio_put(PIN_UHF_CS, 1);
            return RADIO_STATUS_SPI_ERROR;
        }
    }
    gpio_put(PIN_UHF_CS, 1);
    return (written >= 1) ? RADIO_STATUS_OK : RADIO_STATUS_SPI_ERROR;
}

static radio_status_t rfm98_read_fifo(uint8_t *data, uint8_t len) {
    if ((data == NULL) && (len > 0u)) {
        return RADIO_STATUS_INVALID_ARG;
    }
    gpio_put(PIN_UHF_CS, 0);
    uint8_t addr = (uint8_t)(SX127X_REG_FIFO & 0x7Fu);
    int written = spi_write_blocking(UHF_SPI_PORT, &addr, 1);
    if (written != 1) {
        gpio_put(PIN_UHF_CS, 1);
        return RADIO_STATUS_SPI_ERROR;
    }
    if (len > 0u) {
        int read = spi_read_blocking(UHF_SPI_PORT, 0x00, data, len);
        if (read != len) {
            gpio_put(PIN_UHF_CS, 1);
            return RADIO_STATUS_SPI_ERROR;
        }
    }
    gpio_put(PIN_UHF_CS, 1);
    return RADIO_STATUS_OK;
}

static radio_status_t rfm98_set_mode(uint8_t mode) {
    uint8_t op_mode = 0u;
    radio_status_t st = rfm98_read_reg(SX127X_REG_OP_MODE, &op_mode);
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    op_mode |= SX127X_LONG_RANGE_MODE;
    op_mode = (uint8_t)((op_mode & (uint8_t)~0x07u) | (mode & 0x07u));
    return rfm98_write_reg(SX127X_REG_OP_MODE, op_mode);
}

static radio_status_t rfm98_set_frequency(uint32_t freq_hz) {
    uint32_t frf = (uint32_t)(((uint64_t)freq_hz << 19) / 32000000u);
    radio_status_t st = rfm98_write_reg(SX127X_REG_FRF_MSB, (uint8_t)((frf >> 16) & 0xFFu));
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = rfm98_write_reg(SX127X_REG_FRF_MID, (uint8_t)((frf >> 8) & 0xFFu));
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    return rfm98_write_reg(SX127X_REG_FRF_LSB, (uint8_t)(frf & 0xFFu));
}

static radio_status_t rfm98_apply_tx_power(void) {
    int8_t dbm = g_rfm98.tx_power_dbm;
    if (dbm < 2) {
        dbm = 2;
    } else if (dbm > 20) {
        dbm = 20;
    }

    radio_status_t st;
    if (dbm > 17) {
        st = rfm98_write_reg(SX127X_REG_PA_DAC, 0x87);
        if (st != RADIO_STATUS_OK) {
            return st;
        }
        uint8_t output_power = (uint8_t)(dbm - 5);
        if (output_power > 0x0Fu) {
            output_power = 0x0Fu;
        }
        return rfm98_write_reg(SX127X_REG_PA_CONFIG, (uint8_t)(0x80u | output_power));
    }

    st = rfm98_write_reg(SX127X_REG_PA_DAC, 0x84);
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    return rfm98_write_reg(SX127X_REG_PA_CONFIG, (uint8_t)(0x80u | (uint8_t)(dbm - 2)));
}

static radio_status_t rfm98_apply_profile_1(void) {
    radio_status_t st = rfm98_set_mode(SX127X_MODE_STDBY);
    if (st != RADIO_STATUS_OK) {
        return st;
    }

    st = rfm98_set_frequency(438100000u);
    if (st != RADIO_STATUS_OK) {
        return st;
    }

    st = rfm98_write_reg(SX127X_REG_MODEM_CONFIG_1, 0x72); // BW=125kHz, CR=4/5, explicit header
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = rfm98_write_reg(SX127X_REG_MODEM_CONFIG_2, 0x77); // SF7, CRC on, symb timeout msb=3
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = rfm98_write_reg(SX127X_REG_SYMB_TIMEOUT_LSB, 0xFF);
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = rfm98_write_reg(SX127X_REG_MODEM_CONFIG_3, 0x04); // AGC auto on
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = rfm98_write_reg(SX127X_REG_PREAMBLE_MSB, 0x00);
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = rfm98_write_reg(SX127X_REG_PREAMBLE_LSB, 0x08);
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = rfm98_write_reg(SX127X_REG_SYNC_WORD, 0x12);
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = rfm98_write_reg(SX127X_REG_FIFO_TX_BASE_ADDR, 0x00);
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = rfm98_write_reg(SX127X_REG_FIFO_RX_BASE_ADDR, 0x00);
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = rfm98_apply_tx_power();
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    return rfm98_write_reg(SX127X_REG_IRQ_FLAGS, SX127X_IRQ_ALL);
}

static radio_status_t rfm98_read_rx_frame(void) {
    uint8_t current_addr = 0u;
    radio_status_t st = rfm98_read_reg(SX127X_REG_FIFO_RX_CURRENT_ADDR, &current_addr);
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    uint8_t byte_count = 0u;
    st = rfm98_read_reg(SX127X_REG_RX_NB_BYTES, &byte_count);
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    if (byte_count > sizeof(g_rfm98.rx_frame.data)) {
        return RADIO_STATUS_INTERNAL_ERROR;
    }

    st = rfm98_write_reg(SX127X_REG_FIFO_ADDR_PTR, current_addr);
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = rfm98_read_fifo(g_rfm98.rx_frame.data, byte_count);
    if (st != RADIO_STATUS_OK) {
        return st;
    }

    uint8_t snr_raw = 0u;
    uint8_t rssi_raw = 0u;
    st = rfm98_read_reg(SX127X_REG_PKT_SNR_VALUE, &snr_raw);
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = rfm98_read_reg(SX127X_REG_PKT_RSSI_VALUE, &rssi_raw);
    if (st != RADIO_STATUS_OK) {
        return st;
    }

    g_rfm98.rx_frame.length = byte_count;
    g_rfm98.rx_frame.snr_db_x100 = (int16_t)((int16_t)(int8_t)snr_raw * 25);
    g_rfm98.rx_frame.rssi_dbm_x100 = (int16_t)((-164 + (int16_t)rssi_raw) * 100);
    g_rfm98.rx_valid = true;
    return RADIO_STATUS_OK;
}

radio_status_t rfm98pw_init(void) {
    gpio_put(PIN_UHF_RST, 0);
    sleep_ms(2);
    gpio_put(PIN_UHF_RST, 1);
    sleep_ms(8);

    radio_status_t st = rfm98_set_mode(SX127X_MODE_SLEEP);
    if (st != RADIO_STATUS_OK) {
        g_rfm98.state = RFM98_STATE_ERROR;
        return st;
    }
    st = rfm98_set_mode(SX127X_MODE_STDBY);
    if (st != RADIO_STATUS_OK) {
        g_rfm98.state = RFM98_STATE_ERROR;
        return st;
    }

    uint8_t version = 0u;
    st = rfm98_read_reg(SX127X_REG_VERSION, &version);
    if (st != RADIO_STATUS_OK) {
        g_rfm98.state = RFM98_STATE_ERROR;
        return st;
    }
    if (version != 0x12u) {
        g_rfm98.state = RFM98_STATE_ERROR;
        return RADIO_STATUS_NOT_READY;
    }

    g_rfm98.rf_profile = 1u;
    g_rfm98.rx_valid = false;
    g_rfm98.state = RFM98_STATE_IDLE;
    g_rfm98.initialized = true;
    st = rfm98_apply_profile_1();
    if (st != RADIO_STATUS_OK) {
        g_rfm98.state = RFM98_STATE_ERROR;
        g_rfm98.initialized = false;
        return st;
    }
    return RADIO_STATUS_OK;
}

radio_status_t rfm98pw_set_profile(uint8_t rf_profile) {
    if (!g_rfm98.initialized) {
        return RADIO_STATUS_NOT_READY;
    }
    if (rf_profile != 1u) {
        return RADIO_STATUS_INVALID_ARG;
    }
    if ((g_rfm98.state == RFM98_STATE_TX_WAIT) || (g_rfm98.state == RFM98_STATE_RX_WAIT)) {
        return RADIO_STATUS_BUSY;
    }
    g_rfm98.rf_profile = rf_profile;
    return rfm98_apply_profile_1();
}

radio_status_t rfm98pw_set_tx_power_dbm(int8_t dbm) {
    if (!g_rfm98.initialized) {
        return RADIO_STATUS_NOT_READY;
    }
    if ((g_rfm98.state == RFM98_STATE_TX_WAIT) || (g_rfm98.state == RFM98_STATE_RX_WAIT)) {
        return RADIO_STATUS_BUSY;
    }
    g_rfm98.tx_power_dbm = dbm;
    return rfm98_apply_tx_power();
}

radio_status_t rfm98pw_start_tx(const uint8_t *payload, uint8_t length, uint32_t timeout_ms) {
    if (!g_rfm98.initialized) {
        return RADIO_STATUS_NOT_READY;
    }
    if ((payload == NULL) || (length == 0u)) {
        return RADIO_STATUS_INVALID_ARG;
    }
    if (g_rfm98.state != RFM98_STATE_IDLE) {
        return RADIO_STATUS_BUSY;
    }

    radio_status_t st = rfm98_set_mode(SX127X_MODE_STDBY);
    if (st != RADIO_STATUS_OK) {
        g_rfm98.state = RFM98_STATE_ERROR;
        return st;
    }
    st = rfm98_write_reg(SX127X_REG_DIO_MAPPING_1, 0x40); // DIO0 -> TxDone
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = rfm98_write_reg(SX127X_REG_IRQ_FLAGS, SX127X_IRQ_ALL);
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = rfm98_write_reg(SX127X_REG_FIFO_ADDR_PTR, 0x00);
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = rfm98_write_fifo(payload, length);
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = rfm98_write_reg(SX127X_REG_PAYLOAD_LENGTH, length);
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = rfm98_set_mode(SX127X_MODE_TX);
    if (st != RADIO_STATUS_OK) {
        return st;
    }

    set_deadline(timeout_ms);
    g_rfm98.state = RFM98_STATE_TX_WAIT;
    return RADIO_STATUS_OK;
}

radio_status_t rfm98pw_start_rx(uint32_t timeout_ms) {
    if (!g_rfm98.initialized) {
        return RADIO_STATUS_NOT_READY;
    }
    if (g_rfm98.state != RFM98_STATE_IDLE) {
        return RADIO_STATUS_BUSY;
    }

    g_rfm98.rx_valid = false;
    radio_status_t st = rfm98_set_mode(SX127X_MODE_STDBY);
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = rfm98_write_reg(SX127X_REG_DIO_MAPPING_1, 0x00); // DIO0 -> RxDone
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = rfm98_write_reg(SX127X_REG_IRQ_FLAGS, SX127X_IRQ_ALL);
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = rfm98_write_reg(SX127X_REG_FIFO_ADDR_PTR, 0x00);
    if (st != RADIO_STATUS_OK) {
        return st;
    }
    st = rfm98_set_mode(SX127X_MODE_RX_SINGLE);
    if (st != RADIO_STATUS_OK) {
        return st;
    }

    set_deadline(timeout_ms);
    g_rfm98.state = RFM98_STATE_RX_WAIT;
    return RADIO_STATUS_OK;
}

radio_status_t rfm98pw_poll_event(radio_event_t *event) {
    if (event == NULL) {
        return RADIO_STATUS_INVALID_ARG;
    }
    *event = RADIO_EVENT_NONE;

    if (!g_rfm98.initialized) {
        return RADIO_STATUS_NOT_READY;
    }

    if (((g_rfm98.state == RFM98_STATE_TX_WAIT) || (g_rfm98.state == RFM98_STATE_RX_WAIT)) && deadline_expired(g_rfm98.deadline_ms)) {
        (void)rfm98_set_mode(SX127X_MODE_STDBY);
        (void)rfm98_write_reg(SX127X_REG_IRQ_FLAGS, SX127X_IRQ_ALL);
        g_rfm98.state = RFM98_STATE_IDLE;
        *event = RADIO_EVENT_TIMEOUT;
        return RADIO_STATUS_OK;
    }

    uint8_t irq = 0u;
    radio_status_t st = rfm98_read_reg(SX127X_REG_IRQ_FLAGS, &irq);
    if (st != RADIO_STATUS_OK) {
        g_rfm98.state = RFM98_STATE_ERROR;
        *event = RADIO_EVENT_ERROR;
        return st;
    }

    if ((g_rfm98.state == RFM98_STATE_TX_WAIT) && (irq & SX127X_IRQ_TX_DONE)) {
        (void)rfm98_write_reg(SX127X_REG_IRQ_FLAGS, SX127X_IRQ_TX_DONE);
        (void)rfm98_set_mode(SX127X_MODE_STDBY);
        g_rfm98.state = RFM98_STATE_IDLE;
        *event = RADIO_EVENT_TX_DONE;
        return RADIO_STATUS_OK;
    }

    if (g_rfm98.state == RFM98_STATE_RX_WAIT) {
        if (irq & SX127X_IRQ_PAYLOAD_CRC_ERROR) {
            (void)rfm98_write_reg(SX127X_REG_IRQ_FLAGS, SX127X_IRQ_PAYLOAD_CRC_ERROR);
            (void)rfm98_set_mode(SX127X_MODE_STDBY);
            g_rfm98.state = RFM98_STATE_IDLE;
            *event = RADIO_EVENT_CRC_FAIL;
            return RADIO_STATUS_OK;
        }

        if (irq & SX127X_IRQ_RX_TIMEOUT) {
            (void)rfm98_write_reg(SX127X_REG_IRQ_FLAGS, SX127X_IRQ_RX_TIMEOUT);
            (void)rfm98_set_mode(SX127X_MODE_STDBY);
            g_rfm98.state = RFM98_STATE_IDLE;
            *event = RADIO_EVENT_TIMEOUT;
            return RADIO_STATUS_OK;
        }

        if (irq & SX127X_IRQ_RX_DONE) {
            st = rfm98_read_rx_frame();
            if (st != RADIO_STATUS_OK) {
                g_rfm98.state = RFM98_STATE_ERROR;
                *event = RADIO_EVENT_ERROR;
                return st;
            }
            (void)rfm98_write_reg(SX127X_REG_IRQ_FLAGS, SX127X_IRQ_RX_DONE);
            (void)rfm98_set_mode(SX127X_MODE_STDBY);
            g_rfm98.state = RFM98_STATE_RX_READY;
            *event = RADIO_EVENT_RX_DONE;
            return RADIO_STATUS_OK;
        }
    }

    return RADIO_STATUS_OK;
}

radio_status_t rfm98pw_read_rx(radio_rx_frame_t *out_frame) {
    if (out_frame == NULL) {
        return RADIO_STATUS_INVALID_ARG;
    }
    if (!g_rfm98.initialized) {
        return RADIO_STATUS_NOT_READY;
    }
    if ((g_rfm98.state != RFM98_STATE_RX_READY) || !g_rfm98.rx_valid) {
        return RADIO_STATUS_NOT_READY;
    }

    *out_frame = g_rfm98.rx_frame;
    g_rfm98.rx_valid = false;
    g_rfm98.state = RFM98_STATE_IDLE;
    return RADIO_STATUS_OK;
}

radio_status_t rfm98pw_abort(void) {
    if (!g_rfm98.initialized) {
        return RADIO_STATUS_NOT_READY;
    }
    radio_status_t st = rfm98_set_mode(SX127X_MODE_STDBY);
    if (st != RADIO_STATUS_OK) {
        g_rfm98.state = RFM98_STATE_ERROR;
        return st;
    }
    (void)rfm98_write_reg(SX127X_REG_IRQ_FLAGS, SX127X_IRQ_ALL);
    g_rfm98.rx_valid = false;
    g_rfm98.state = RFM98_STATE_IDLE;
    return RADIO_STATUS_OK;
}
