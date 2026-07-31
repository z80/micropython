#include "nrf24.h"
#include "nrf24_regs.h"
#include <string.h>

// Helper to handle simple command reads
static uint8_t nrf24_cmd_read(nrf24_t *self, uint8_t cmd) {
    uint8_t tx[1] = {cmd};
    uint8_t rx[1] = {0};
    self->csn_set(self, 0);
    self->spi_transfer(self, 1, tx, rx);
    self->csn_set(self, 1);
    return rx[0];
}

uint8_t nrf24_reg_read(nrf24_t *self, uint8_t reg) {
    uint8_t tx[2] = {reg & 0x1F, 0x00};
    uint8_t rx[2] = {0};
    self->csn_set(self, 0);
    self->spi_transfer(self, 2, tx, rx);
    self->csn_set(self, 1);
    return rx[1];
}

uint8_t nrf24_reg_write(nrf24_t *self, uint8_t reg, uint8_t value) {
    uint8_t tx[2] = {0x20 | reg, value};
    uint8_t rx[2] = {0};
    self->csn_set(self, 0);
    self->spi_transfer(self, 2, tx, rx);
    self->csn_set(self, 1);
    return rx[0];
}

uint8_t nrf24_reg_write_bytes(nrf24_t *self, uint8_t reg, const uint8_t *buf, size_t len) {
    uint8_t cmd = 0x20 | reg;
    uint8_t status = 0;
    self->csn_set(self, 0);
    self->spi_transfer(self, 1, &cmd, &status);
    // Transmit buffer; we don't care about the RX bytes here
    self->spi_transfer(self, len, buf, NULL); 
    self->csn_set(self, 1);
    return status;
}

uint8_t nrf24_read_status(nrf24_t *self) {
    return nrf24_cmd_read(self, NOP);
}

void nrf24_flush_rx(nrf24_t *self) { nrf24_cmd_read(self, FLUSH_RX); }
void nrf24_flush_tx(nrf24_t *self) { nrf24_cmd_read(self, FLUSH_TX); }

void nrf24_power_up(nrf24_t *self) {
    if (self->powered) return;
    uint8_t config = nrf24_reg_read(self, CONFIG);
    nrf24_reg_write(self, CONFIG, config | PWR_UP);
    self->delay_us(1500);
    self->powered = true;
}

void nrf24_power_down(nrf24_t *self) {
    self->ce_set(self, 0);
    uint8_t config = nrf24_reg_read(self, CONFIG);
    nrf24_reg_write(self, CONFIG, config & ~PWR_UP);
    self->powered = false;
}

void nrf24_set_power_speed(nrf24_t *self, uint8_t power, uint8_t speed) {
    uint8_t setup = nrf24_reg_read(self, RF_SETUP) & 0b11010001;
    nrf24_reg_write(self, RF_SETUP, setup | power | speed);
}

void nrf24_set_channel(nrf24_t *self, uint8_t channel) {
    if (channel > 125) channel = 125;
    nrf24_reg_write(self, RF_CH, channel);
}

void nrf24_init(nrf24_t *self, uint8_t channel, uint8_t payload_size, uint8_t speed, uint8_t power, uint16_t ard_us, uint8_t arc) {
    self->payload_size = payload_size;
    self->powered = false;
    self->has_pipe0_addr = false;

    self->delay_us(5000);
    
    nrf24_reg_write(self, SETUP_AW, 0b11);
    nrf24_reg_write(self, DYNPD, 0);

    uint8_t ard_steps = ard_us / 250;
    if (ard_steps > 0) ard_steps--;
    if (ard_steps > 15) ard_steps = 15;
    
    nrf24_reg_write(self, SETUP_RETR, (ard_steps << 4) | (arc & 0x0F));
    nrf24_set_power_speed(self, power, speed);
    
    // set_crc(2) logic
    uint8_t config = nrf24_reg_read(self, CONFIG) & ~(CRCO | EN_CRC);
    nrf24_reg_write(self, CONFIG, config | EN_CRC | CRCO);
    
    nrf24_reg_write(self, STATUS, RX_DR | TX_DS | MAX_RT);
    nrf24_set_channel(self, channel);
    nrf24_flush_rx(self);
    nrf24_flush_tx(self);
    nrf24_power_up(self);
}

void nrf24_open_tx_pipe(nrf24_t *self, const uint8_t *address) {
    nrf24_reg_write_bytes(self, RX_ADDR_P0, address, 5);
    nrf24_reg_write_bytes(self, TX_ADDR, address, 5);
    nrf24_reg_write(self, RX_PW_P0, self->payload_size);
    nrf24_reg_write(self, EN_RXADDR, nrf24_reg_read(self, EN_RXADDR) | 0x01);
}

void nrf24_start_listening(nrf24_t *self) {
    nrf24_power_up(self);
    uint8_t config = nrf24_reg_read(self, CONFIG);
    nrf24_reg_write(self, CONFIG, config | PRIM_RX);
    nrf24_reg_write(self, STATUS, RX_DR | TX_DS | MAX_RT);

    if (self->has_pipe0_addr) {
        nrf24_reg_write_bytes(self, RX_ADDR_P0, self->pipe0_read_addr, 5);
    }
    self->ce_set(self, 1);
    self->delay_us(130);
}

void nrf24_send_start(nrf24_t *self, const uint8_t *buf, size_t len) {
    nrf24_power_up(self);
    uint8_t config = nrf24_reg_read(self, CONFIG);
    nrf24_reg_write(self, CONFIG, (config | PWR_UP) & ~PRIM_RX);
    nrf24_reg_write(self, STATUS, RX_DR | TX_DS | MAX_RT);

    uint8_t cmd = W_TX_PAYLOAD;
    self->csn_set(self, 0);
    self->spi_transfer(self, 1, &cmd, NULL);
    self->spi_transfer(self, len, buf, NULL);
    
    // Pad with zeros if payload is less than configured size
    if (len < self->payload_size) {
        uint8_t pad = 0;
        for (size_t i = len; i < self->payload_size; i++) {
            self->spi_transfer(self, 1, &pad, NULL);
        }
    }
    self->csn_set(self, 1);

    // Pulse CE
    self->ce_set(self, 1);
    self->delay_us(15);
    self->ce_set(self, 0);
}

int8_t nrf24_send_done(nrf24_t *self) {
    uint8_t status = nrf24_read_status(self);
    if (!(status & (TX_DS | MAX_RT))) return 0; // In progress
    
    nrf24_reg_write(self, STATUS, RX_DR | TX_DS | MAX_RT); // Clear flags
    if (status & TX_DS) return 1; // Success
    return 2; // Failure (MAX_RT)
}

void nrf24_abort_send(nrf24_t *self) {
    self->ce_set(self, 0);
    nrf24_flush_tx(self);
    nrf24_reg_write(self, STATUS, TX_DS | MAX_RT);
}

int8_t nrf24_send(nrf24_t *self, const uint8_t *buf, size_t len, uint32_t timeout_ms) {
    nrf24_send_start(self, buf, len);
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
