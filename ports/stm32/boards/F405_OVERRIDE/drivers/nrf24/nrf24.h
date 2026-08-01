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

typedef enum {
    NRF24_SEND_TIMEOUT = -1,
    NRF24_SEND_INVALID = -2,
    NRF24_SEND_PENDING = 0,
    NRF24_SEND_OK = 1,
    NRF24_SEND_MAX_RT = 2,
} nrf24_send_result_t;

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
uint8_t nrf24_reg_read_bytes(nrf24_t *self, uint8_t reg, uint8_t *buf, size_t len);
uint8_t nrf24_reg_write_bytes(nrf24_t *self, uint8_t reg, const uint8_t *buf, size_t len);
uint8_t nrf24_read_status(nrf24_t *self);
/* IRQ sources use the NRF24_STATUS_RX_DR/TX_DS/MAX_RT bit values. */
void nrf24_set_irq_sources(nrf24_t *self, uint8_t sources);
void nrf24_clear_irq(nrf24_t *self, uint8_t sources);
uint8_t nrf24_read_observe_tx(nrf24_t *self);
uint8_t nrf24_read_fifo_status(nrf24_t *self);
bool nrf24_rx_fifo_empty(nrf24_t *self);
bool nrf24_tx_fifo_empty(nrf24_t *self);
bool nrf24_tx_fifo_full(nrf24_t *self);
void nrf24_flush_rx(nrf24_t *self);
void nrf24_flush_tx(nrf24_t *self);

// Core API
bool nrf24_init(nrf24_t *self, uint8_t channel, uint8_t payload_size, uint8_t speed, uint8_t power, uint16_t ard_us, uint8_t arc);
void nrf24_power_up(nrf24_t *self);
void nrf24_power_down(nrf24_t *self);
void nrf24_set_power_speed(nrf24_t *self, uint8_t power, uint8_t speed);
void nrf24_set_channel(nrf24_t *self, uint8_t channel);
void nrf24_open_tx_pipe(nrf24_t *self, const uint8_t *address);
void nrf24_open_rx_pipe(nrf24_t *self, uint8_t pipe_id, const uint8_t *address);
void nrf24_close_rx_pipe(nrf24_t *self, uint8_t pipe_id);

// RX / TX
void nrf24_start_listening(nrf24_t *self);
void nrf24_stop_listening(nrf24_t *self);
/* Non-blocking mode primitives for IRQ-driven users.  RX asserts CE; TX
   leaves CE low so the caller can fill the FIFO before asserting it. */
void nrf24_enter_rx_mode(nrf24_t *self);
void nrf24_enter_tx_mode(nrf24_t *self);
void nrf24_set_ce(nrf24_t *self, bool high);
bool nrf24_any(nrf24_t *self);
bool nrf24_read_payload(nrf24_t *self, uint8_t *buf, size_t capacity, size_t *out_len);
bool nrf24_read_payload_ex(nrf24_t *self, uint8_t *buf, size_t capacity,
    size_t *out_len, uint8_t *status_out);
bool nrf24_write_payload(nrf24_t *self, const uint8_t *buf, size_t len);
bool nrf24_recv(nrf24_t *self, uint8_t *buf);
bool nrf24_send_start(nrf24_t *self, const uint8_t *buf, size_t len);
int8_t nrf24_send_done(nrf24_t *self); // returns 0 (in progress), 1 (success), 2 (fail)
int8_t nrf24_send(nrf24_t *self, const uint8_t *buf, size_t len, uint32_t timeout_ms);
void nrf24_restart_tx(nrf24_t *self);
void nrf24_abort_send(nrf24_t *self);

#endif // NRF24_H
