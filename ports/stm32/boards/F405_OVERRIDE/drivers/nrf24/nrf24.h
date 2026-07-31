#ifndef NRF24_H
#define NRF24_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct _nrf24_t nrf24_t;

// Hardware Abstraction Function Pointers
typedef void (*nrf24_spi_transfer_t)(nrf24_t *self, size_t len, const uint8_t *src, uint8_t *dest);
typedef void (*nrf24_pin_set_t)(nrf24_t *self, uint8_t state);
typedef void (*nrf24_delay_us_t)(uint32_t us);
typedef uint32_t (*nrf24_ticks_ms_t)(void); // Needed for blocking send timeouts

struct _nrf24_t {
    // HAL callbacks
    nrf24_spi_transfer_t spi_transfer;
    nrf24_pin_set_t      ce_set;
    nrf24_pin_set_t      csn_set;
    nrf24_delay_us_t     delay_us;
    nrf24_ticks_ms_t     ticks_ms;

    void *user_data; 
    
    // State matching python object
    uint8_t payload_size;
    bool powered;
    uint8_t pipe0_read_addr[5];
    bool has_pipe0_addr;
};

// Internal/Register functions
uint8_t nrf24_reg_read(nrf24_t *self, uint8_t reg);
uint8_t nrf24_reg_write(nrf24_t *self, uint8_t reg, uint8_t value);
uint8_t nrf24_reg_write_bytes(nrf24_t *self, uint8_t reg, const uint8_t *buf, size_t len);
uint8_t nrf24_read_status(nrf24_t *self);
void nrf24_flush_rx(nrf24_t *self);
void nrf24_flush_tx(nrf24_t *self);

// Core API
void nrf24_init(nrf24_t *self, uint8_t channel, uint8_t payload_size, uint8_t speed, uint8_t power, uint16_t ard_us, uint8_t arc);
void nrf24_power_up(nrf24_t *self);
void nrf24_power_down(nrf24_t *self);
void nrf24_set_power_speed(nrf24_t *self, uint8_t power, uint8_t speed);
void nrf24_set_channel(nrf24_t *self, uint8_t channel);
void nrf24_open_tx_pipe(nrf24_t *self, const uint8_t *address);
void nrf24_open_rx_pipe(nrf24_t *self, uint8_t pipe_id, const uint8_t *address);

// RX / TX
void nrf24_start_listening(nrf24_t *self);
void nrf24_stop_listening(nrf24_t *self);
bool nrf24_any(nrf24_t *self);
void nrf24_recv(nrf24_t *self, uint8_t *buf);
void nrf24_send_start(nrf24_t *self, const uint8_t *buf, size_t len);
int8_t nrf24_send_done(nrf24_t *self); // returns 0 (in progress), 1 (success), 2 (fail)
int8_t nrf24_send(nrf24_t *self, const uint8_t *buf, size_t len, uint32_t timeout_ms);
void nrf24_abort_send(nrf24_t *self);

#endif // NRF24_H

