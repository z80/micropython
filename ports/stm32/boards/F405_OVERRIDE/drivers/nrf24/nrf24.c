#include "nrf24.h"
#include "nrf24_regs.h"
#include <string.h>

// Helper to handle simple command reads
static uint8_t nrf24_cmd_read(nrf24_t *self, uint8_t cmd) {
    uint8_t tx[1];
    uint8_t rx[1] = {0};
    tx[0] = cmd;
    self->csn_set(self, 0);
    self->spi_transfer(self, 1, tx, rx);
    self->csn_set(self, 1);
    return rx[0];
}

uint8_t nrf24_reg_read(nrf24_t *self, uint8_t reg) {
    uint8_t tx[2];
    uint8_t rx[2] = {0};
    tx[0] = reg & 0x1f;
    tx[1] = 0;
    self->csn_set(self, 0);
    self->spi_transfer(self, 2, tx, rx);
    self->csn_set(self, 1);
    return rx[1];
}

uint8_t nrf24_reg_write(nrf24_t *self, uint8_t reg, uint8_t value) {
    uint8_t tx[2];
    uint8_t rx[2] = {0};
    tx[0] = 0x20 | reg;
    tx[1] = value;
    self->csn_set(self, 0);
    self->spi_transfer(self, 2, tx, rx);
    self->csn_set(self, 1);
    return rx[0];
}

uint8_t nrf24_reg_read_bytes(nrf24_t *self, uint8_t reg, uint8_t *buf, size_t len) {
    uint8_t tx[NRF24_MAX_PAYLOAD + 1];
    uint8_t rx[NRF24_MAX_PAYLOAD + 1];
    if (buf == NULL || len > NRF24_MAX_PAYLOAD) return 0;
    tx[0] = reg & 0x1F;
    memset(tx + 1, NRF24_CMD_NOP, len);
    self->csn_set(self, 0);
    self->spi_transfer(self, len + 1, tx, rx);
    self->csn_set(self, 1);
    memcpy(buf, rx + 1, len);
    return rx[0];
}

uint8_t nrf24_reg_write_bytes(nrf24_t *self, uint8_t reg, const uint8_t *buf, size_t len) {
    uint8_t tx[NRF24_MAX_PAYLOAD + 1];
    uint8_t rx[NRF24_MAX_PAYLOAD + 1];
    if (buf == NULL || len > NRF24_MAX_PAYLOAD) return 0;
    tx[0] = 0x20 | reg;
    memcpy(tx + 1, buf, len);
    self->csn_set(self, 0);
    self->spi_transfer(self, len + 1, tx, rx);
    self->csn_set(self, 1);
    return rx[0];
}

uint8_t nrf24_read_status(nrf24_t *self) {
    return nrf24_cmd_read(self, NRF24_CMD_NOP);
}

void nrf24_set_irq_sources(nrf24_t *self, uint8_t sources) {
    const uint8_t all_sources = NRF24_STATUS_RX_DR |
        NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT;
    uint8_t config = nrf24_reg_read(self, NRF24_REG_CONFIG);
    uint8_t disabled = (uint8_t)(~sources) & all_sources;
    config = (config & (uint8_t)~all_sources) | disabled;
    nrf24_reg_write(self, NRF24_REG_CONFIG, config);
}

void nrf24_clear_irq(nrf24_t *self, uint8_t sources) {
    nrf24_reg_write(self, NRF24_REG_STATUS,
        sources & (NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS |
            NRF24_STATUS_MAX_RT));
}

uint8_t nrf24_read_observe_tx(nrf24_t *self) {
    return nrf24_reg_read(self, NRF24_REG_OBSERVE_TX);
}

uint8_t nrf24_read_fifo_status(nrf24_t *self) {
    return nrf24_reg_read(self, NRF24_REG_FIFO_STATUS);
}

bool nrf24_rx_fifo_empty(nrf24_t *self) {
    return (nrf24_read_fifo_status(self) & NRF24_FIFO_RX_EMPTY) != 0;
}

bool nrf24_tx_fifo_empty(nrf24_t *self) {
    return (nrf24_read_fifo_status(self) & NRF24_FIFO_TX_EMPTY) != 0;
}

bool nrf24_tx_fifo_full(nrf24_t *self) {
    return (nrf24_read_fifo_status(self) & NRF24_FIFO_TX_FULL) != 0;
}

void nrf24_flush_rx(nrf24_t *self) { nrf24_cmd_read(self, NRF24_CMD_FLUSH_RX); }
void nrf24_flush_tx(nrf24_t *self) { nrf24_cmd_read(self, NRF24_CMD_FLUSH_TX); }

void nrf24_power_up(nrf24_t *self) {
    if (self->powered) return;
    uint8_t config = nrf24_reg_read(self, NRF24_REG_CONFIG);
    nrf24_reg_write(self, NRF24_REG_CONFIG, config | NRF24_CFG_PWR_UP);
    self->delay_us(1500);
    self->powered = true;
}

void nrf24_power_down(nrf24_t *self) {
    self->ce_set(self, 0);
    uint8_t config = nrf24_reg_read(self, NRF24_REG_CONFIG);
    nrf24_reg_write(self, NRF24_REG_CONFIG, config & ~NRF24_CFG_PWR_UP);
    self->powered = false;
}

void nrf24_set_power_speed(nrf24_t *self, uint8_t power, uint8_t speed) {
    uint8_t setup = nrf24_reg_read(self, NRF24_REG_RF_SETUP) & 0xd1;
    nrf24_reg_write(self, NRF24_REG_RF_SETUP, setup | power | speed);
}

void nrf24_set_channel(nrf24_t *self, uint8_t channel) {
    if (channel > 125) channel = 125;
    nrf24_reg_write(self, NRF24_REG_RF_CH, channel);
}

bool nrf24_init(nrf24_t *self, uint8_t channel, uint8_t payload_size, uint8_t speed, uint8_t power, uint16_t ard_us, uint8_t arc) {
    if (self == NULL || payload_size == 0 || payload_size > NRF24_MAX_PAYLOAD)
        return false;
    self->payload_size = payload_size;
    self->powered = false;
    self->has_pipe0_addr = false;

    self->delay_us(5000);
    
    nrf24_reg_write(self, NRF24_REG_SETUP_AW, 0x03);
    nrf24_reg_write(self, NRF24_REG_EN_AA, 0x3f);
    nrf24_reg_write(self, NRF24_REG_EN_RXADDR, 0x00);
    nrf24_reg_write(self, NRF24_REG_DYNPD, 0);

    uint8_t ard_steps = (uint8_t)(ard_us / 250u);
    if (ard_steps > 0) ard_steps--;
    if (ard_steps > 15) ard_steps = 15;
    
    nrf24_reg_write(self, NRF24_REG_SETUP_RETR, (ard_steps << 4) | (arc & 0x0F));
    nrf24_set_power_speed(self, power, speed);
    
    // set_crc(2) logic
    uint8_t config = nrf24_reg_read(self, NRF24_REG_CONFIG) &
        (uint8_t)~(NRF24_CFG_CRCO | NRF24_CFG_EN_CRC);
    nrf24_reg_write(self, NRF24_REG_CONFIG, config | NRF24_CFG_EN_CRC | NRF24_CFG_CRCO);
    nrf24_set_irq_sources(self,
        NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);
    
    nrf24_clear_irq(self,
        NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);
    nrf24_set_channel(self, channel);
    nrf24_flush_rx(self);
    nrf24_flush_tx(self);
    nrf24_power_up(self);
    return true;
}

void nrf24_open_tx_pipe(nrf24_t *self, const uint8_t *address) {
    nrf24_reg_write_bytes(self, NRF24_REG_RX_ADDR_P0, address, NRF24_ADDR_LEN);
    nrf24_reg_write_bytes(self, NRF24_REG_TX_ADDR, address, NRF24_ADDR_LEN);
    nrf24_reg_write(self, NRF24_REG_RX_PW_P0, self->payload_size);
    nrf24_reg_write(self, NRF24_REG_EN_RXADDR, nrf24_reg_read(self, NRF24_REG_EN_RXADDR) | 0x01);
}

void nrf24_open_rx_pipe(nrf24_t *self, uint8_t pipe_id, const uint8_t *address) {
    if (pipe_id > 5 || address == NULL) return;
    if (pipe_id == 0) {
        memcpy(self->pipe0_read_addr, address, NRF24_ADDR_LEN);
        self->has_pipe0_addr = true;
    }
    if (pipe_id < 2) {
        nrf24_reg_write_bytes(self, NRF24_REG_RX_ADDR_P0 + pipe_id,
            address, NRF24_ADDR_LEN);
    } else {
        nrf24_reg_write(self, NRF24_REG_RX_ADDR_P0 + pipe_id, address[0]);
    }
    nrf24_reg_write(self, NRF24_REG_RX_PW_P0 + pipe_id, self->payload_size);
    nrf24_reg_write(self, NRF24_REG_EN_RXADDR,
        nrf24_reg_read(self, NRF24_REG_EN_RXADDR) | (1u << pipe_id));
}

void nrf24_close_rx_pipe(nrf24_t *self, uint8_t pipe_id) {
    if (pipe_id > 5) return;
    nrf24_reg_write(self, NRF24_REG_EN_RXADDR,
        nrf24_reg_read(self, NRF24_REG_EN_RXADDR) & ~(1u << pipe_id));
    if (pipe_id == 0) self->has_pipe0_addr = false;
}

void nrf24_set_ce(nrf24_t *self, bool high) {
    if (self != NULL && self->ce_set != NULL) self->ce_set(self, high ? 1u : 0u);
}

void nrf24_enter_rx_mode(nrf24_t *self) {
    nrf24_power_up(self);
    uint8_t config = nrf24_reg_read(self, NRF24_REG_CONFIG);
    nrf24_reg_write(self, NRF24_REG_CONFIG, config | NRF24_CFG_PRIM_RX);
    nrf24_clear_irq(self, NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);

    if (self->has_pipe0_addr) {
        nrf24_reg_write_bytes(self, NRF24_REG_RX_ADDR_P0, self->pipe0_read_addr, NRF24_ADDR_LEN);
        nrf24_reg_write(self, NRF24_REG_EN_RXADDR,
            nrf24_reg_read(self, NRF24_REG_EN_RXADDR) | 0x01);
    } else {
        nrf24_reg_write(self, NRF24_REG_EN_RXADDR,
            nrf24_reg_read(self, NRF24_REG_EN_RXADDR) & (uint8_t)~0x01);
    }
    nrf24_set_ce(self, true);
}

void nrf24_start_listening(nrf24_t *self) {
    nrf24_enter_rx_mode(self);
    self->delay_us(130);
}

void nrf24_enter_tx_mode(nrf24_t *self) {
    nrf24_set_ce(self, false);
    nrf24_power_up(self);
    uint8_t config = nrf24_reg_read(self, NRF24_REG_CONFIG);
    nrf24_reg_write(self, NRF24_REG_CONFIG,
        (config | NRF24_CFG_PWR_UP) & (uint8_t)~NRF24_CFG_PRIM_RX);
    nrf24_clear_irq(self, NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);
}

void nrf24_stop_listening(nrf24_t *self) {
    nrf24_enter_tx_mode(self);
}

bool nrf24_any(nrf24_t *self) {
    return !nrf24_rx_fifo_empty(self);
}

bool nrf24_write_payload(nrf24_t *self, const uint8_t *buf, size_t len) {
    uint8_t tx[NRF24_MAX_PAYLOAD + 1];
    uint8_t rx[NRF24_MAX_PAYLOAD + 1];
    if ((buf == NULL && len != 0) || len > self->payload_size ||
            self->payload_size > NRF24_MAX_PAYLOAD || nrf24_tx_fifo_full(self)) {
        return false;
    }
    tx[0] = NRF24_CMD_W_TX_PAYLOAD;
    if (len != 0) memcpy(tx + 1, buf, len);
    memset(tx + 1 + len, 0, self->payload_size - len);
    self->csn_set(self, 0);
    self->spi_transfer(self, self->payload_size + 1, tx, rx);
    self->csn_set(self, 1);
    return true;
}

bool nrf24_read_payload_ex(nrf24_t *self, uint8_t *buf, size_t capacity,
        size_t *out_len, uint8_t *status_out) {
    uint8_t tx[NRF24_MAX_PAYLOAD + 1];
    uint8_t rx[NRF24_MAX_PAYLOAD + 1];
    if (buf == NULL || capacity < self->payload_size ||
            self->payload_size > NRF24_MAX_PAYLOAD || nrf24_rx_fifo_empty(self)) {
        return false;
    }
    memset(tx, NRF24_CMD_NOP, self->payload_size + 1);
    tx[0] = NRF24_CMD_R_RX_PAYLOAD;
    self->csn_set(self, 0);
    self->spi_transfer(self, self->payload_size + 1, tx, rx);
    self->csn_set(self, 1);
    memcpy(buf, rx + 1, self->payload_size);
    if (out_len != NULL) *out_len = self->payload_size;
    if (status_out != NULL) *status_out = rx[0];
    return true;
}

bool nrf24_read_payload(nrf24_t *self, uint8_t *buf, size_t capacity,
        size_t *out_len) {
    return nrf24_read_payload_ex(self, buf, capacity, out_len, NULL);
}

bool nrf24_recv(nrf24_t *self, uint8_t *buf) {
    size_t ignored;
    bool read = nrf24_read_payload(self, buf, self->payload_size, &ignored);
    if (read && nrf24_rx_fifo_empty(self)) nrf24_clear_irq(self, NRF24_STATUS_RX_DR);
    return read;
}

bool nrf24_send_start(nrf24_t *self, const uint8_t *buf, size_t len) {
    nrf24_power_up(self);
    uint8_t config = nrf24_reg_read(self, NRF24_REG_CONFIG);
    nrf24_reg_write(self, NRF24_REG_CONFIG, (config | NRF24_CFG_PWR_UP) & ~NRF24_CFG_PRIM_RX);
    nrf24_clear_irq(self, NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);

    if (!nrf24_write_payload(self, buf, len)) return false;

    // Pulse CE
    self->ce_set(self, 1);
    self->delay_us(15);
    self->ce_set(self, 0);
    return true;
}

int8_t nrf24_send_done(nrf24_t *self) {
    uint8_t status = nrf24_read_status(self);
    uint8_t completed = status & (NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);
    if (completed == 0) return NRF24_SEND_PENDING;
    nrf24_clear_irq(self, completed);
    return (completed & NRF24_STATUS_MAX_RT) != 0
        ? NRF24_SEND_MAX_RT : NRF24_SEND_OK;
}

void nrf24_restart_tx(nrf24_t *self) {
    nrf24_clear_irq(self, NRF24_STATUS_MAX_RT);
    self->ce_set(self, 1);
    self->delay_us(15);
    self->ce_set(self, 0);
}

void nrf24_abort_send(nrf24_t *self) {
    self->ce_set(self, 0);
    nrf24_flush_tx(self);
    nrf24_reg_write(self, NRF24_REG_STATUS, NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);
}

int8_t nrf24_send(nrf24_t *self, const uint8_t *buf, size_t len, uint32_t timeout_ms) {
    if (!nrf24_send_start(self, buf, len)) return NRF24_SEND_INVALID;
    uint32_t start = self->ticks_ms();
    
    while (true) {
        int8_t res = nrf24_send_done(self);
        if (res != 0) {
            if (res == 2) nrf24_flush_tx(self);
            return res; 
        }
        if ((self->ticks_ms() - start) >= timeout_ms) {
            nrf24_abort_send(self);
            return -1; // Timeout
        }
        self->delay_us(50);
    }
}
