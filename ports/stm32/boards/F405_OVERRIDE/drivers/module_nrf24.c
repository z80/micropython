#include "py/obj.h"
#include "py/runtime.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "extmod/modmachine.h"
#include "extmod/virtpin.h"

#include "nrf24_regs.h"
#include <string.h>

#ifndef STATIC
#define STATIC static
#endif

#ifndef MODULE_NRF24_ENABLED
#define MODULE_NRF24_ENABLED 1
#endif

#if MODULE_NRF24_ENABLED

typedef struct _mp_nrf24_obj_t {
    mp_obj_base_t base;
    mp_obj_t spi;
    mp_hal_pin_obj_t csn;
    mp_hal_pin_obj_t ce;
    mp_hal_pin_obj_t irq_pin;   /* 0 / unused if none */
    uint8_t payload_size;
    uint8_t channel;
    bool powered;
    bool listening;
    bool streaming;
    uint8_t pipe0_addr[NRF24_ADDR_LEN];
    bool pipe0_addr_valid;

    /* stream ring: payload-only chunks of payload_size (or app length) */
    uint8_t *tx_ring;
    size_t tx_ring_size;
    volatile size_t tx_head;
    volatile size_t tx_tail;
    volatile uint32_t tx_ok;
    volatile uint32_t max_rt;
    volatile uint32_t overflow;
    uint8_t stream_policy; /* 0=stop, 1=drop_continue, 2=retry */
    uint8_t in_flight;
} mp_nrf24_obj_t;

/* ---------- SPI / pins ---------- */

STATIC void csn_low(mp_nrf24_obj_t *self)  { mp_hal_pin_write(self->csn, 0); }
STATIC void csn_high(mp_nrf24_obj_t *self) { mp_hal_pin_write(self->csn, 1); }
STATIC void ce_low(mp_nrf24_obj_t *self)   { mp_hal_pin_write(self->ce, 0); }
STATIC void ce_high(mp_nrf24_obj_t *self)  { mp_hal_pin_write(self->ce, 1); }


STATIC void spi_transfer(mp_nrf24_obj_t * self, size_t len, const uint8_t *tx, uint8_t *rx) {
    mp_obj_base_t *s = (mp_obj_base_t *)MP_OBJ_TO_PTR(self->spi);
    
    // Retrieve the protocol table from the SPI object type
    mp_machine_spi_p_t *spi_p = (mp_machine_spi_p_t *)MP_OBJ_TYPE_GET_SLOT(s->type, protocol);
    
    if (spi_p && spi_p->transfer) {
        spi_p->transfer(s, len, tx, rx);
    }
}

STATIC uint8_t nrf_read_reg(mp_nrf24_obj_t *self, uint8_t reg) {
    uint8_t tx[2] = { reg, NRF24_CMD_NOP };
    uint8_t rx[2] = { 0, 0 };
    csn_low(self);
    spi_transfer(self, 2, tx, rx);
    csn_high(self);
    return rx[1];
}

STATIC void nrf_write_reg(mp_nrf24_obj_t *self, uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { (uint8_t)(0x20 | reg), val };
    uint8_t rx[2];
    csn_low(self);
    spi_transfer(self, 2, tx, rx);
    csn_high(self);
}

STATIC void nrf_write_reg_bytes(mp_nrf24_obj_t *self, uint8_t reg, const uint8_t *buf, size_t len) {
    uint8_t cmd = (uint8_t)(0x20 | reg);
    csn_low(self);
    spi_transfer(self, 1, &cmd, NULL);
    spi_transfer(self, len, buf, NULL);
    csn_high(self);
}

STATIC uint8_t nrf_status(mp_nrf24_obj_t *self) {
    uint8_t tx = NRF24_CMD_NOP, rx = 0;
    csn_low(self);
    spi_transfer(self, 1, &tx, &rx);
    csn_high(self);
    return rx;
}

STATIC void nrf_cmd(mp_nrf24_obj_t *self, uint8_t cmd) {
    csn_low(self);
    spi_transfer(self, 1, &cmd, NULL);
    csn_high(self);
}

STATIC void nrf_clear_status(mp_nrf24_obj_t *self, uint8_t bits) {
    nrf_write_reg(self, NRF24_REG_STATUS, bits);
}

/* ---------- power / mode ---------- */

STATIC void nrf_power_up(mp_nrf24_obj_t *self) {
    if (self->powered) {
        return;
    }
    uint8_t c = nrf_read_reg(self, NRF24_REG_CONFIG);
    nrf_write_reg(self, NRF24_REG_CONFIG, (uint8_t)(c | NRF24_CFG_PWR_UP));
    mp_hal_delay_us(1500);
    self->powered = true;
}

STATIC void nrf_start_listening(mp_nrf24_obj_t *self) {
    nrf_power_up(self);
    uint8_t c = nrf_read_reg(self, NRF24_REG_CONFIG);
    nrf_write_reg(self, NRF24_REG_CONFIG, (uint8_t)(c | NRF24_CFG_PRIM_RX | NRF24_CFG_PWR_UP));
    nrf_clear_status(self, NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);
    if (self->pipe0_addr_valid) {
        nrf_write_reg_bytes(self, NRF24_REG_RX_ADDR_P0, self->pipe0_addr, NRF24_ADDR_LEN);
    }
    ce_high(self);
    mp_hal_delay_us(130);
    self->listening = true;
}

STATIC void nrf_stop_listening(mp_nrf24_obj_t *self) {
    ce_low(self);
    self->listening = false;
    /* stay powered */
}

/* ---------- TX one packet (blocking path) ---------- */

STATIC void nrf_write_payload(mp_nrf24_obj_t *self, const uint8_t *data, size_t len) {
    uint8_t cmd = NRF24_CMD_W_TX_PAYLOAD;
    uint8_t pad = 0;
    csn_low(self);
    spi_transfer(self, 1, &cmd, NULL);
    if (len > self->payload_size) {
        len = self->payload_size;
    }
    spi_transfer(self, len, data, NULL);
    for (size_t i = len; i < self->payload_size; i++) {
        spi_transfer(self, 1, &pad, NULL);
    }
    csn_high(self);
}

STATIC int nrf_send_blocking(mp_nrf24_obj_t *self, const uint8_t *data, size_t len, uint32_t timeout_ms) {
    nrf_power_up(self);
    nrf_stop_listening(self);

    uint8_t c = nrf_read_reg(self, NRF24_REG_CONFIG);
    nrf_write_reg(self, NRF24_REG_CONFIG, (uint8_t)((c | NRF24_CFG_PWR_UP) & ~NRF24_CFG_PRIM_RX));
    nrf_clear_status(self, NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);

    nrf_write_payload(self, data, len);

    ce_high(self);
    mp_hal_delay_us(15);
    ce_low(self);

    uint32_t start = mp_hal_ticks_ms();
    for (;;) {
        uint8_t st = nrf_status(self);
        if (st & NRF24_STATUS_TX_DS) {
            nrf_clear_status(self, NRF24_STATUS_TX_DS);
            return 0;
        }
        if (st & NRF24_STATUS_MAX_RT) {
            nrf_cmd(self, NRF24_CMD_FLUSH_TX);
            nrf_clear_status(self, NRF24_STATUS_MAX_RT);
            return -1;
        }
        if ((mp_hal_ticks_ms() - start) >= timeout_ms) {
            nrf_cmd(self, NRF24_CMD_FLUSH_TX);
            return -2;
        }
        mp_hal_delay_us(50);
    }
}

STATIC bool nrf_any(mp_nrf24_obj_t *self) {
    return (nrf_read_reg(self, NRF24_REG_FIFO_STATUS) & NRF24_FIFO_RX_EMPTY) == 0;
}

STATIC int nrf_recv(mp_nrf24_obj_t *self, uint8_t *out, size_t max_len) {
    if (!nrf_any(self)) {
        return 0;
    }
    size_t n = self->payload_size;
    if (n > max_len) {
        n = max_len;
    }
    uint8_t cmd = NRF24_CMD_R_RX_PAYLOAD;
    csn_low(self);
    spi_transfer(self, 1, &cmd, NULL);
    spi_transfer(self, n, NULL, out);
    /* discard rest if max_len < payload_size */
    uint8_t discard;
    for (size_t i = n; i < self->payload_size; i++) {
        spi_transfer(self, 1, NULL, &discard);
    }
    csn_high(self);
    nrf_clear_status(self, NRF24_STATUS_RX_DR);
    return (int)n;
}

/* ---------- stream ring (payload bytes, full radio frames later in transport) ---------- */

STATIC size_t ring_used(mp_nrf24_obj_t *self) {
    size_t h = self->tx_head, t = self->tx_tail;
    return (h >= t) ? (h - t) : (self->tx_ring_size - t + h);
}

STATIC size_t ring_free(mp_nrf24_obj_t *self) {
    return self->tx_ring_size - 1 - ring_used(self);
}

STATIC bool tx_fifo_full(mp_nrf24_obj_t *self) {
    return (nrf_read_reg(self, NRF24_REG_FIFO_STATUS) & NRF24_FIFO_TX_FULL) != 0;
}

/* Push one full payload_size frame from ring into nRF TX FIFO */
STATIC bool stream_push_one(mp_nrf24_obj_t *self) {
    if (ring_used(self) < self->payload_size || tx_fifo_full(self)) {
        return false;
    }
    uint8_t frame[NRF24_MAX_PAYLOAD];
    size_t t = self->tx_tail;
    for (size_t i = 0; i < self->payload_size; i++) {
        frame[i] = self->tx_ring[t];
        t = (t + 1) % self->tx_ring_size;
    }
    self->tx_tail = t;
    nrf_write_payload(self, frame, self->payload_size);
    if (self->in_flight < 3) {
        self->in_flight++;
    }
    return true;
}

/* Call from IRQ or poll: service TX_DS / MAX_RT and refill */
void nrf24_stream_service(mp_nrf24_obj_t *self) {
    if (!self->streaming) {
        return;
    }
    uint8_t st = nrf_status(self);

    if (st & NRF24_STATUS_MAX_RT) {
        self->max_rt++;
        nrf_cmd(self, NRF24_CMD_FLUSH_TX);
        nrf_clear_status(self, NRF24_STATUS_MAX_RT | NRF24_STATUS_TX_DS);
        self->in_flight = 0;
        if (self->stream_policy == 0) { /* STOP */
            self->streaming = false;
            ce_low(self);
            return;
        }
        /* DROP_CONTINUE: discard in-flight notion, keep feeding new ring data */
        if (self->stream_policy == 2) {
            /* RETRY: same as drop after flush for simplicity in v1 */
        }
    }

    if (st & NRF24_STATUS_TX_DS) {
        nrf_clear_status(self, NRF24_STATUS_TX_DS);
        self->tx_ok++;
        if (self->in_flight) {
            self->in_flight--;
        }
    }

    while (stream_push_one(self)) {
        /* fill HW FIFO */
    }
    if (ring_used(self) >= self->payload_size || self->in_flight) {
        ce_high(self);
    }
}

/* ---------- Python bindings ---------- */

STATIC mp_obj_t nrf24_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum { ARG_spi, ARG_csn, ARG_ce, ARG_channel, ARG_payload_size };
    static const mp_arg_t allowed[] = {
        { MP_QSTR_spi, MP_ARG_REQUIRED | MP_ARG_OBJ, {} },
        { MP_QSTR_csn, MP_ARG_REQUIRED | MP_ARG_OBJ, {} },
        { MP_QSTR_ce,  MP_ARG_REQUIRED | MP_ARG_OBJ, {} },
        { MP_QSTR_channel, MP_ARG_INT, {.u_int = 46} },
        { MP_QSTR_payload_size, MP_ARG_INT, {.u_int = 32} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed), allowed, args);

    mp_nrf24_obj_t *self = m_new_obj(mp_nrf24_obj_t);
    memset(self, 0, sizeof(*self));
    self->base.type = type;
    self->spi = args[ARG_spi].u_obj;
    self->csn = mp_hal_get_pin_obj(args[ARG_csn].u_obj);
    self->ce  = mp_hal_get_pin_obj(args[ARG_ce].u_obj);
    self->channel = (uint8_t)args[ARG_channel].u_int;
    self->payload_size = (uint8_t)args[ARG_payload_size].u_int;
    if (self->payload_size == 0 || self->payload_size > NRF24_MAX_PAYLOAD) {
        mp_raise_ValueError(MP_ERROR_TEXT("payload_size"));
    }

    mp_hal_pin_output(self->csn);
    mp_hal_pin_output(self->ce);
    csn_high(self);
    ce_low(self);

    mp_hal_delay_ms(5);
    nrf_write_reg(self, NRF24_REG_SETUP_AW, 0x03);
    if (nrf_read_reg(self, NRF24_REG_SETUP_AW) != 0x03) {
        mp_raise_OSError(MP_ENODEV);
    }
    nrf_write_reg(self, NRF24_REG_DYNPD, 0);
    nrf_write_reg(self, NRF24_REG_SETUP_RETR, (uint8_t)((6 << 4) | 8)); /* 1750 us, 8 retries */
    nrf_write_reg(self, NRF24_REG_RF_SETUP, (uint8_t)(NRF24_POWER_3 | NRF24_SPEED_2M));
    uint8_t cfg = nrf_read_reg(self, NRF24_REG_CONFIG);
    nrf_write_reg(self, NRF24_REG_CONFIG, (uint8_t)(cfg | NRF24_CFG_EN_CRC | NRF24_CFG_CRCO));
    nrf_write_reg(self, NRF24_REG_RF_CH, self->channel);
    nrf_clear_status(self, NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);
    nrf_cmd(self, NRF24_CMD_FLUSH_RX);
    nrf_cmd(self, NRF24_CMD_FLUSH_TX);
    nrf_power_up(self);

    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t nrf24_start_listening(mp_obj_t self_in) {
    nrf_start_listening(MP_OBJ_TO_PTR(self_in));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(nrf24_start_listening_obj, nrf24_start_listening);

STATIC mp_obj_t nrf24_stop_listening(mp_obj_t self_in) {
    nrf_stop_listening(MP_OBJ_TO_PTR(self_in));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(nrf24_stop_listening_obj, nrf24_stop_listening);

STATIC mp_obj_t nrf24_any(mp_obj_t self_in) {
    return mp_obj_new_bool(nrf_any(MP_OBJ_TO_PTR(self_in)));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(nrf24_any_obj, nrf24_any);

STATIC mp_obj_t nrf24_recv(mp_obj_t self_in) {
    mp_nrf24_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint8_t buf[NRF24_MAX_PAYLOAD];
    int n = nrf_recv(self, buf, self->payload_size);
    if (n <= 0) {
        return mp_const_none;
    }
    return mp_obj_new_bytes(buf, (size_t)n);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(nrf24_recv_obj, nrf24_recv);

STATIC mp_obj_t nrf24_send(mp_obj_t self_in, mp_obj_t data_in) {
    mp_nrf24_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_in, &bufinfo, MP_BUFFER_READ);
    int r = nrf_send_blocking(self, bufinfo.buf, bufinfo.len, 50);
    if (r == -1) {
        mp_raise_OSError(MP_EIO);
    }
    if (r == -2) {
        mp_raise_OSError(MP_ETIMEDOUT);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(nrf24_send_obj, nrf24_send);

STATIC mp_obj_t nrf24_open_tx_pipe(mp_obj_t self_in, mp_obj_t addr_in) {
    mp_nrf24_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_buffer_info_t a;
    mp_get_buffer_raise(addr_in, &a, MP_BUFFER_READ);
    if (a.len != NRF24_ADDR_LEN) {
        mp_raise_ValueError(MP_ERROR_TEXT("addr must be 5 bytes"));
    }
    nrf_write_reg_bytes(self, NRF24_REG_RX_ADDR_P0, a.buf, NRF24_ADDR_LEN);
    nrf_write_reg_bytes(self, NRF24_REG_TX_ADDR, a.buf, NRF24_ADDR_LEN);
    nrf_write_reg(self, NRF24_REG_RX_PW_P0, self->payload_size);
    uint8_t en = nrf_read_reg(self, NRF24_REG_EN_RXADDR);
    nrf_write_reg(self, NRF24_REG_EN_RXADDR, (uint8_t)(en | 0x01));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(nrf24_open_tx_pipe_obj, nrf24_open_tx_pipe);

STATIC mp_obj_t nrf24_open_rx_pipe(mp_obj_t self_in, mp_obj_t pipe_in, mp_obj_t addr_in) {
    mp_nrf24_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int pipe = mp_obj_get_int(pipe_in);
    mp_buffer_info_t a;
    mp_get_buffer_raise(addr_in, &a, MP_BUFFER_READ);
    if (pipe < 0 || pipe > 5 || a.len != NRF24_ADDR_LEN) {
        mp_raise_ValueError(MP_ERROR_TEXT("pipe/addr"));
    }
    if (pipe == 0) {
        memcpy(self->pipe0_addr, a.buf, NRF24_ADDR_LEN);
        self->pipe0_addr_valid = true;
    }
    if (pipe < 2) {
        nrf_write_reg_bytes(self, (uint8_t)(NRF24_REG_RX_ADDR_P0 + pipe), a.buf, NRF24_ADDR_LEN);
    } else {
        nrf_write_reg(self, (uint8_t)(NRF24_REG_RX_ADDR_P0 + pipe), ((uint8_t *)a.buf)[0]);
    }
    nrf_write_reg(self, (uint8_t)(NRF24_REG_RX_PW_P0 + pipe), self->payload_size);
    uint8_t en = nrf_read_reg(self, NRF24_REG_EN_RXADDR);
    nrf_write_reg(self, NRF24_REG_EN_RXADDR, (uint8_t)(en | (1 << pipe)));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(nrf24_open_rx_pipe_obj, nrf24_open_rx_pipe);

/* stream_start(addr, ring_size=2048, policy=1) */
STATIC mp_obj_t nrf24_stream_start(size_t n_args, const mp_obj_t *pos, mp_map_t *kw) {
    enum { ARG_addr, ARG_ring_size, ARG_policy };
    static const mp_arg_t allowed[] = {
        { MP_QSTR_addr, MP_ARG_REQUIRED | MP_ARG_OBJ, {} },
        { MP_QSTR_ring_size, MP_ARG_INT, {.u_int = 2048} },
        { MP_QSTR_policy, MP_ARG_INT, {.u_int = 1} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed)];
    mp_arg_parse_all(n_args - 1, pos + 1, kw, MP_ARRAY_SIZE(allowed), allowed, args);
    mp_nrf24_obj_t *self = MP_OBJ_TO_PTR(pos[0]);

    mp_buffer_info_t a;
    mp_get_buffer_raise(args[ARG_addr].u_obj, &a, MP_BUFFER_READ);
    if (a.len != NRF24_ADDR_LEN) {
        mp_raise_ValueError(MP_ERROR_TEXT("addr"));
    }

    size_t rs = (size_t)args[ARG_ring_size].u_int;
    if (rs < self->payload_size * 4) {
        mp_raise_ValueError(MP_ERROR_TEXT("ring_size"));
    }
    self->tx_ring = m_new(uint8_t, rs);
    self->tx_ring_size = rs;
    self->tx_head = self->tx_tail = 0;
    self->tx_ok = self->max_rt = self->overflow = 0;
    self->in_flight = 0;
    self->stream_policy = (uint8_t)args[ARG_policy].u_int;

    nrf_stop_listening(self);
    /* reuse open_tx_pipe logic */
    nrf_write_reg_bytes(self, NRF24_REG_RX_ADDR_P0, a.buf, NRF24_ADDR_LEN);
    nrf_write_reg_bytes(self, NRF24_REG_TX_ADDR, a.buf, NRF24_ADDR_LEN);
    nrf_write_reg(self, NRF24_REG_RX_PW_P0, self->payload_size);

    uint8_t c = nrf_read_reg(self, NRF24_REG_CONFIG);
    nrf_write_reg(self, NRF24_REG_CONFIG, (uint8_t)((c | NRF24_CFG_PWR_UP) & ~NRF24_CFG_PRIM_RX));
    nrf_clear_status(self, NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);
    nrf_cmd(self, NRF24_CMD_FLUSH_TX);

    self->streaming = true;
    ce_high(self);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(nrf24_stream_start_obj, 2, nrf24_stream_start);

/* stream_write(buf) -> bytes accepted */
STATIC mp_obj_t nrf24_stream_write(mp_obj_t self_in, mp_obj_t data_in) {
    mp_nrf24_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (!self->streaming || !self->tx_ring) {
        mp_raise_ValueError(MP_ERROR_TEXT("not streaming"));
    }
    mp_buffer_info_t b;
    mp_get_buffer_raise(data_in, &b, MP_BUFFER_READ);

    /* Critical section would mask IRQ when you add EXTI */
    size_t free = ring_free(self);
    size_t n = b.len < free ? b.len : free;
    if (n < b.len) {
        self->overflow++;
    }
    const uint8_t *p = b.buf;
    size_t h = self->tx_head;
    for (size_t i = 0; i < n; i++) {
        self->tx_ring[h] = p[i];
        h = (h + 1) % self->tx_ring_size;
    }
    self->tx_head = h;

    nrf24_stream_service(self);
    return MP_OBJ_NEW_SMALL_INT(n);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(nrf24_stream_write_obj, nrf24_stream_write);

STATIC mp_obj_t nrf24_stream_service_py(mp_obj_t self_in) {
    nrf24_stream_service(MP_OBJ_TO_PTR(self_in));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(nrf24_stream_service_obj, nrf24_stream_service_py);

STATIC mp_obj_t nrf24_stream_status(mp_obj_t self_in) {
    mp_nrf24_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_obj_t tuple[6] = {
        mp_obj_new_bool(self->streaming),
        MP_OBJ_NEW_SMALL_INT(ring_used(self)),
        MP_OBJ_NEW_SMALL_INT(self->in_flight),
        MP_OBJ_NEW_SMALL_INT(self->tx_ok),
        MP_OBJ_NEW_SMALL_INT(self->max_rt),
        MP_OBJ_NEW_SMALL_INT(self->overflow),
    };
    return mp_obj_new_tuple(6, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(nrf24_stream_status_obj, nrf24_stream_status);

STATIC mp_obj_t nrf24_stream_stop(mp_obj_t self_in) {
    mp_nrf24_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->streaming = false;
    ce_low(self);
    nrf_cmd(self, NRF24_CMD_FLUSH_TX);
    nrf_clear_status(self, NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);
    if (self->tx_ring) {
        m_del(uint8_t, self->tx_ring, self->tx_ring_size);
        self->tx_ring = NULL;
        self->tx_ring_size = 0;
    }
    self->in_flight = 0;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(nrf24_stream_stop_obj, nrf24_stream_stop);

STATIC const mp_rom_map_elem_t nrf24_locals[] = {
    { MP_ROM_QSTR(MP_QSTR_start_listening), MP_ROM_PTR(&nrf24_start_listening_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop_listening), MP_ROM_PTR(&nrf24_stop_listening_obj) },
    { MP_ROM_QSTR(MP_QSTR_any), MP_ROM_PTR(&nrf24_any_obj) },
    { MP_ROM_QSTR(MP_QSTR_recv), MP_ROM_PTR(&nrf24_recv_obj) },
    { MP_ROM_QSTR(MP_QSTR_send), MP_ROM_PTR(&nrf24_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_open_tx_pipe), MP_ROM_PTR(&nrf24_open_tx_pipe_obj) },
    { MP_ROM_QSTR(MP_QSTR_open_rx_pipe), MP_ROM_PTR(&nrf24_open_rx_pipe_obj) },
    { MP_ROM_QSTR(MP_QSTR_stream_start), MP_ROM_PTR(&nrf24_stream_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_stream_write), MP_ROM_PTR(&nrf24_stream_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_stream_service), MP_ROM_PTR(&nrf24_stream_service_obj) },
    { MP_ROM_QSTR(MP_QSTR_stream_status), MP_ROM_PTR(&nrf24_stream_status_obj) },
    { MP_ROM_QSTR(MP_QSTR_stream_stop), MP_ROM_PTR(&nrf24_stream_stop_obj) },
};
STATIC MP_DEFINE_CONST_DICT(nrf24_locals_dict, nrf24_locals);

STATIC MP_DEFINE_CONST_OBJ_TYPE(
    nrf24_type,
    MP_QSTR_NRF24,
    MP_TYPE_FLAG_NONE,
    make_new, nrf24_make_new,
    locals_dict, &nrf24_locals_dict
);

STATIC const mp_rom_map_elem_t nrf24_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_nrf24) },
    { MP_ROM_QSTR(MP_QSTR_NRF24), MP_ROM_PTR(&nrf24_type) },
};
STATIC MP_DEFINE_CONST_DICT(nrf24_module_globals, nrf24_module_globals_table);

const mp_obj_module_t nrf24_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&nrf24_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_nrf24, nrf24_user_cmodule);

#endif /* MODULE_NRF24_ENABLED */


