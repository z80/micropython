#include "py/obj.h"
#include "py/runtime.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "extmod/modmachine.h"
#include "extmod/virtpin.h"

#include "nrf24.h"
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
    nrf24_t nrf;
    mp_obj_t spi;
    mp_hal_pin_obj_t csn;
    mp_hal_pin_obj_t ce;
    mp_hal_pin_obj_t irq_pin;
} mp_nrf24_obj_t;

/* ---------- HAL Callbacks ---------- */

#define GET_SELF(nrf_ptr) ((mp_nrf24_obj_t *)((char *)(nrf_ptr) - offsetof(mp_nrf24_obj_t, nrf)))

STATIC void cb_csn_set(nrf24_t *nrf, uint8_t state) {
    mp_hal_pin_write(GET_SELF(nrf)->csn, state);
}

STATIC void cb_ce_set(nrf24_t *nrf, uint8_t state) {
    mp_hal_pin_write(GET_SELF(nrf)->ce, state);
}

STATIC void cb_spi_transfer(nrf24_t *nrf, size_t len, const uint8_t *tx, uint8_t *rx) {
    mp_nrf24_obj_t *self = GET_SELF(nrf);
    mp_obj_base_t *s = (mp_obj_base_t *)MP_OBJ_TO_PTR(self->spi);
    mp_machine_spi_p_t *spi_p = (mp_machine_spi_p_t *)MP_OBJ_TYPE_GET_SLOT(s->type, protocol);
    if (spi_p && spi_p->transfer) {
        spi_p->transfer(s, len, (const uint8_t *)tx, rx);
    }
}

STATIC void cb_delay_us(uint32_t us) {
    mp_hal_delay_us(us);
}

STATIC uint32_t cb_ticks_ms(void) {
    return mp_hal_ticks_ms();
}

/* ---------- Python Bindings ---------- */

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
    
    uint8_t channel = (uint8_t)args[ARG_channel].u_int;
    uint8_t payload_size = (uint8_t)args[ARG_payload_size].u_int;
    
    if (payload_size == 0 || payload_size > NRF24_MAX_PAYLOAD) {
        mp_raise_ValueError(MP_ERROR_TEXT("payload_size"));
    }

    // Bind callbacks
    self->nrf.csn_set = cb_csn_set;
    self->nrf.ce_set = cb_ce_set;
    self->nrf.spi_transfer = cb_spi_transfer;
    self->nrf.delay_us = cb_delay_us;
    self->nrf.ticks_ms = cb_ticks_ms;

    mp_hal_pin_output(self->csn);
    mp_hal_pin_output(self->ce);

    // Initialize core driver
    nrf24_init(&self->nrf, channel, payload_size, NRF24_SPEED_2M, NRF24_POWER_3, 1750, 8);

    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t mp_nrf24_start_listening(mp_obj_t self_in) {
    mp_nrf24_obj_t *self = MP_OBJ_TO_PTR(self_in);
    nrf24_start_listening(&self->nrf);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(nrf24_start_listening_obj, mp_nrf24_start_listening);

STATIC mp_obj_t mp_nrf24_stop_listening(mp_obj_t self_in) {
    mp_nrf24_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->nrf.ce_set(&self->nrf, 0);
    uint8_t config = nrf24_reg_read(&self->nrf, NRF24_REG_CONFIG);
    nrf24_reg_write(&self->nrf, NRF24_REG_CONFIG, config & ~NRF24_CFG_PRIM_RX);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(nrf24_stop_listening_obj, mp_nrf24_stop_listening);

STATIC mp_obj_t mp_nrf24_any(mp_obj_t self_in) {
    mp_nrf24_obj_t *self = MP_OBJ_TO_PTR(self_in);
    bool rx_avail = (nrf24_reg_read(&self->nrf, NRF24_REG_FIFO_STATUS) & NRF24_FIFO_RX_EMPTY) == 0;
    return mp_obj_new_bool(rx_avail);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(nrf24_any_obj, mp_nrf24_any);

STATIC mp_obj_t mp_nrf24_recv(mp_obj_t self_in) {
    mp_nrf24_obj_t *self = MP_OBJ_TO_PTR(self_in);
    
    if (nrf24_reg_read(&self->nrf, NRF24_REG_FIFO_STATUS) & NRF24_FIFO_RX_EMPTY) {
        return mp_const_none;
    }
    
    uint8_t buf[NRF24_MAX_PAYLOAD];
    size_t n = self->nrf.payload_size;
    
    uint8_t cmd = NRF24_CMD_R_RX_PAYLOAD;
    self->nrf.csn_set(&self->nrf, 0);
    self->nrf.spi_transfer(&self->nrf, 1, &cmd, NULL);
    self->nrf.spi_transfer(&self->nrf, n, NULL, buf);
    self->nrf.csn_set(&self->nrf, 1);
    
    nrf24_reg_write(&self->nrf, NRF24_REG_STATUS, NRF24_STATUS_RX_DR);
    return mp_obj_new_bytes(buf, n);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(nrf24_recv_obj, mp_nrf24_recv);

STATIC mp_obj_t mp_nrf24_send(mp_obj_t self_in, mp_obj_t data_in) {
    mp_nrf24_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_in, &bufinfo, MP_BUFFER_READ);
    
    int8_t r = nrf24_send(&self->nrf, bufinfo.buf, bufinfo.len, 50);
    if (r == -1) {
        mp_raise_OSError(MP_ETIMEDOUT);
    }
    if (r == 2) {
        mp_raise_OSError(MP_EIO); // MAX_RT failure
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(nrf24_send_obj, mp_nrf24_send);

STATIC mp_obj_t mp_nrf24_open_tx_pipe(mp_obj_t self_in, mp_obj_t addr_in) {
    mp_nrf24_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_buffer_info_t a;
    mp_get_buffer_raise(addr_in, &a, MP_BUFFER_READ);
    
    if (a.len != NRF24_ADDR_LEN) {
        mp_raise_ValueError(MP_ERROR_TEXT("addr must be 5 bytes"));
    }
    
    nrf24_open_tx_pipe(&self->nrf, a.buf);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(nrf24_open_tx_pipe_obj, mp_nrf24_open_tx_pipe);

STATIC mp_obj_t mp_nrf24_open_rx_pipe(mp_obj_t self_in, mp_obj_t pipe_in, mp_obj_t addr_in) {
    mp_nrf24_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int pipe = mp_obj_get_int(pipe_in);
    mp_buffer_info_t a;
    
    mp_get_buffer_raise(addr_in, &a, MP_BUFFER_READ);
    if (pipe < 0 || pipe > 5 || a.len != NRF24_ADDR_LEN) {
        mp_raise_ValueError(MP_ERROR_TEXT("pipe/addr"));
    }
    
    if (pipe == 0) {
        memcpy(self->nrf.pipe0_read_addr, a.buf, NRF24_ADDR_LEN);
        self->nrf.has_pipe0_addr = true;
    }
    
    if (pipe < 2) {
        nrf24_reg_write_bytes(&self->nrf, NRF24_REG_RX_ADDR_P0 + pipe, a.buf, NRF24_ADDR_LEN);
    } else {
        nrf24_reg_write(&self->nrf, NRF24_REG_RX_ADDR_P0 + pipe, ((uint8_t *)a.buf)[0]);
    }
    
    nrf24_reg_write(&self->nrf, NRF24_REG_RX_PW_P0 + pipe, self->nrf.payload_size);
    uint8_t en = nrf24_reg_read(&self->nrf, NRF24_REG_EN_RXADDR);
    nrf24_reg_write(&self->nrf, NRF24_REG_EN_RXADDR, en | (1 << pipe));
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(nrf24_open_rx_pipe_obj, mp_nrf24_open_rx_pipe);

STATIC const mp_rom_map_elem_t nrf24_locals[] = {
    { MP_ROM_QSTR(MP_QSTR_start_listening), MP_ROM_PTR(&nrf24_start_listening_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop_listening), MP_ROM_PTR(&nrf24_stop_listening_obj) },
    { MP_ROM_QSTR(MP_QSTR_any), MP_ROM_PTR(&nrf24_any_obj) },
    { MP_ROM_QSTR(MP_QSTR_recv), MP_ROM_PTR(&nrf24_recv_obj) },
    { MP_ROM_QSTR(MP_QSTR_send), MP_ROM_PTR(&nrf24_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_open_tx_pipe), MP_ROM_PTR(&nrf24_open_tx_pipe_obj) },
    { MP_ROM_QSTR(MP_QSTR_open_rx_pipe), MP_ROM_PTR(&nrf24_open_rx_pipe_obj) },
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

