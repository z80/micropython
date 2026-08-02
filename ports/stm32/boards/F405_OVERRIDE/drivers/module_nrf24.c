#include "py/obj.h"
#include "py/runtime.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "extmod/modmachine.h"
#include "extmod/virtpin.h"

#include "nrf24.h"
#include "nrf24_regs.h"
#include "module_nrf24.h"
#include <string.h>

#ifndef STATIC
#define STATIC static
#endif

#ifndef MODULE_NRF24_ENABLED
#define MODULE_NRF24_ENABLED 1
#endif

#if MODULE_NRF24_ENABLED

STATIC const mp_obj_type_t nrf24_type;

typedef struct _mp_nrf24_obj_t {
    mp_obj_base_t base;
    nrf24_t nrf;
    mp_obj_t spi;
    mp_obj_t csn_obj;
    mp_obj_t ce_obj;
    mp_hal_pin_obj_t csn;
    mp_hal_pin_obj_t ce;
    void *native_owner;
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
    mp_machine_spi_p_t *spi_p = (mp_machine_spi_p_t *)
        MP_OBJ_TYPE_GET_SLOT_OR_NULL(s->type, protocol);
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

STATIC mp_nrf24_obj_t *nrf24_obj_checked(mp_obj_t object) {
    if (!mp_obj_is_type(object, &nrf24_type)) {
        mp_raise_TypeError(MP_ERROR_TEXT("NRF24 required"));
    }
    return MP_OBJ_TO_PTR(object);
}

STATIC mp_nrf24_obj_t *nrf24_obj_for_python(mp_obj_t object) {
    mp_nrf24_obj_t *self = nrf24_obj_checked(object);
    if (self->native_owner != NULL) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("radio in use"));
    }
    return self;
}

nrf24_t *mp_nrf24_get_native(mp_obj_t radio_obj) {
    return &nrf24_obj_checked(radio_obj)->nrf;
}

bool mp_nrf24_claim(mp_obj_t radio_obj, void *owner) {
    mp_nrf24_obj_t *self = nrf24_obj_checked(radio_obj);
    if (owner == NULL || self->native_owner != NULL) return false;
    self->native_owner = owner;
    return true;
}

bool mp_nrf24_release(mp_obj_t radio_obj, void *owner) {
    mp_nrf24_obj_t *self = nrf24_obj_checked(radio_obj);
    if (owner == NULL || self->native_owner != owner) return false;
    self->native_owner = NULL;
    return true;
}

bool mp_nrf24_is_claimed(mp_obj_t radio_obj) {
    return nrf24_obj_checked(radio_obj)->native_owner != NULL;
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
    self->csn_obj = args[ARG_csn].u_obj;
    self->ce_obj = args[ARG_ce].u_obj;
    self->csn = mp_hal_get_pin_obj(args[ARG_csn].u_obj);
    self->ce  = mp_hal_get_pin_obj(args[ARG_ce].u_obj);

    mp_obj_base_t *spi_base = (mp_obj_base_t *)MP_OBJ_TO_PTR(self->spi);
    mp_machine_spi_p_t *spi_protocol = (mp_machine_spi_p_t *)
        MP_OBJ_TYPE_GET_SLOT_OR_NULL(spi_base->type, protocol);
    if (spi_protocol == NULL || spi_protocol->transfer == NULL) {
        mp_raise_TypeError(MP_ERROR_TEXT("SPI object required"));
    }
    
    mp_int_t channel_arg = args[ARG_channel].u_int;
    mp_int_t payload_arg = args[ARG_payload_size].u_int;
    
    if (channel_arg < 0 || channel_arg > 125) {
        mp_raise_ValueError(MP_ERROR_TEXT("channel"));
    }
    if (payload_arg <= 0 || payload_arg > NRF24_MAX_PAYLOAD) {
        mp_raise_ValueError(MP_ERROR_TEXT("payload_size"));
    }
    uint8_t channel = (uint8_t)channel_arg;
    uint8_t payload_size = (uint8_t)payload_arg;

    // Bind callbacks
    self->nrf.csn_set = cb_csn_set;
    self->nrf.ce_set = cb_ce_set;
    self->nrf.spi_transfer = cb_spi_transfer;
    self->nrf.delay_us = cb_delay_us;
    self->nrf.ticks_ms = cb_ticks_ms;

    mp_hal_pin_output(self->csn);
    mp_hal_pin_output(self->ce);
    mp_hal_pin_write(self->csn, 1);
    mp_hal_pin_write(self->ce, 0);

    // Initialize core driver
    if (!nrf24_init(&self->nrf, channel, payload_size,
            NRF24_SPEED_2M, NRF24_POWER_3, 1750, 8)) {
        mp_raise_ValueError(MP_ERROR_TEXT("radio configuration"));
    }

    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t mp_nrf24_start_listening(mp_obj_t self_in) {
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
    nrf24_start_listening(&self->nrf);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(nrf24_start_listening_obj, mp_nrf24_start_listening);

STATIC mp_obj_t mp_nrf24_stop_listening(mp_obj_t self_in) {
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
    nrf24_stop_listening(&self->nrf);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(nrf24_stop_listening_obj, mp_nrf24_stop_listening);

STATIC mp_obj_t mp_nrf24_any(mp_obj_t self_in) {
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
    return mp_obj_new_bool(nrf24_any(&self->nrf));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(nrf24_any_obj, mp_nrf24_any);

STATIC mp_obj_t mp_nrf24_recv(mp_obj_t self_in) {
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
    uint8_t buf[NRF24_MAX_PAYLOAD];
    if (!nrf24_recv(&self->nrf, buf)) {
        return mp_const_none;
    }
    return mp_obj_new_bytes(buf, self->nrf.payload_size);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(nrf24_recv_obj, mp_nrf24_recv);

STATIC mp_obj_t mp_nrf24_send(mp_obj_t self_in, mp_obj_t data_in) {
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_in, &bufinfo, MP_BUFFER_READ);
    
    if (bufinfo.len > self->nrf.payload_size) {
        mp_raise_ValueError(MP_ERROR_TEXT("payload length"));
    }
    int8_t r = nrf24_send(&self->nrf, bufinfo.buf, bufinfo.len, 50);
    if (r == -1) {
        mp_raise_OSError(MP_ETIMEDOUT);
    }
    if (r == NRF24_SEND_INVALID) {
        mp_raise_OSError(MP_EBUSY);
    }
    if (r == 2) {
        mp_raise_OSError(MP_EIO); // MAX_RT failure
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(nrf24_send_obj, mp_nrf24_send);

STATIC mp_obj_t mp_nrf24_open_tx_pipe(mp_obj_t self_in, mp_obj_t addr_in) {
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
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
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
    int pipe = mp_obj_get_int(pipe_in);
    mp_buffer_info_t a;
    
    mp_get_buffer_raise(addr_in, &a, MP_BUFFER_READ);
    if (pipe < 0 || pipe > 5 || a.len != NRF24_ADDR_LEN) {
        mp_raise_ValueError(MP_ERROR_TEXT("pipe/addr"));
    }
    
    nrf24_open_rx_pipe(&self->nrf, (uint8_t)pipe, a.buf);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(nrf24_open_rx_pipe_obj, mp_nrf24_open_rx_pipe);

STATIC mp_obj_t mp_nrf24_close_rx_pipe(mp_obj_t self_in, mp_obj_t pipe_in) {
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
    mp_int_t pipe = mp_obj_get_int(pipe_in);
    if (pipe < 0 || pipe > 5) mp_raise_ValueError(MP_ERROR_TEXT("pipe"));
    nrf24_close_rx_pipe(&self->nrf, (uint8_t)pipe);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(nrf24_close_rx_pipe_obj, mp_nrf24_close_rx_pipe);

STATIC mp_obj_t mp_nrf24_recv_into(mp_obj_t self_in, mp_obj_t buffer_in) {
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
    mp_buffer_info_t buffer;
    mp_get_buffer_raise(buffer_in, &buffer, MP_BUFFER_WRITE);
    if (buffer.len < self->nrf.payload_size) {
        mp_raise_ValueError(MP_ERROR_TEXT("buffer too small"));
    }
    size_t length = 0;
    if (!nrf24_read_payload(&self->nrf, buffer.buf, buffer.len, &length)) {
        return MP_OBJ_NEW_SMALL_INT(0);
    }
    if (nrf24_rx_fifo_empty(&self->nrf)) nrf24_clear_irq(&self->nrf, NRF24_STATUS_RX_DR);
    return MP_OBJ_NEW_SMALL_INT(length);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(nrf24_recv_into_obj, mp_nrf24_recv_into);


/* ---------- Additional Method Wrappers ---------- */

STATIC mp_obj_t mp_nrf24_power_up(mp_obj_t self_in) {
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
    nrf24_power_up(&self->nrf);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(nrf24_power_up_obj, mp_nrf24_power_up);

STATIC mp_obj_t mp_nrf24_power_down(mp_obj_t self_in) {
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
    nrf24_power_down(&self->nrf);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(nrf24_power_down_obj, mp_nrf24_power_down);

STATIC mp_obj_t mp_nrf24_set_power_speed(mp_obj_t self_in, mp_obj_t power_in, mp_obj_t speed_in) {
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
    mp_int_t power = mp_obj_get_int(power_in);
    mp_int_t speed = mp_obj_get_int(speed_in);
    if ((power != NRF24_POWER_0 && power != NRF24_POWER_1 &&
            power != NRF24_POWER_2 && power != NRF24_POWER_3) ||
            (speed != NRF24_SPEED_250K && speed != NRF24_SPEED_1M &&
            speed != NRF24_SPEED_2M)) {
        mp_raise_ValueError(MP_ERROR_TEXT("power/speed"));
    }
    nrf24_set_power_speed(&self->nrf, (uint8_t)power, (uint8_t)speed);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(nrf24_set_power_speed_obj, mp_nrf24_set_power_speed);

STATIC mp_obj_t mp_nrf24_set_channel(mp_obj_t self_in, mp_obj_t channel_in) {
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
    mp_int_t channel = mp_obj_get_int(channel_in);
    if (channel < 0 || channel > 125) mp_raise_ValueError(MP_ERROR_TEXT("channel"));
    nrf24_set_channel(&self->nrf, (uint8_t)channel);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(nrf24_set_channel_obj, mp_nrf24_set_channel);

STATIC mp_obj_t mp_nrf24_reg_read(mp_obj_t self_in, mp_obj_t reg_in) {
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
    uint8_t val = nrf24_reg_read(&self->nrf, (uint8_t)mp_obj_get_int(reg_in));
    return MP_OBJ_NEW_SMALL_INT(val);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(nrf24_reg_read_obj, mp_nrf24_reg_read);

STATIC mp_obj_t mp_nrf24_reg_write(mp_obj_t self_in, mp_obj_t reg_in, mp_obj_t val_in) {
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
    uint8_t res = nrf24_reg_write(&self->nrf, (uint8_t)mp_obj_get_int(reg_in), (uint8_t)mp_obj_get_int(val_in));
    return MP_OBJ_NEW_SMALL_INT(res);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(nrf24_reg_write_obj, mp_nrf24_reg_write);

STATIC mp_obj_t mp_nrf24_read_status(mp_obj_t self_in) {
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
    return MP_OBJ_NEW_SMALL_INT(nrf24_read_status(&self->nrf));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(nrf24_read_status_obj, mp_nrf24_read_status);

STATIC uint8_t mp_nrf24_irq_sources(mp_obj_t sources_in) {
    mp_int_t sources = mp_obj_get_int(sources_in);
    const uint8_t valid = NRF24_STATUS_RX_DR |
        NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT;
    if (sources < 0 || (sources & ~valid) != 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("IRQ sources"));
    }
    return (uint8_t)sources;
}

STATIC mp_obj_t mp_nrf24_set_irq_sources(mp_obj_t self_in,
        mp_obj_t sources_in) {
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
    nrf24_set_irq_sources(&self->nrf, mp_nrf24_irq_sources(sources_in));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(nrf24_set_irq_sources_obj,
    mp_nrf24_set_irq_sources);

STATIC mp_obj_t mp_nrf24_clear_irq(mp_obj_t self_in, mp_obj_t sources_in) {
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
    nrf24_clear_irq(&self->nrf, mp_nrf24_irq_sources(sources_in));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(nrf24_clear_irq_obj, mp_nrf24_clear_irq);

STATIC mp_obj_t mp_nrf24_read_observe_tx(mp_obj_t self_in) {
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
    return MP_OBJ_NEW_SMALL_INT(nrf24_read_observe_tx(&self->nrf));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(nrf24_read_observe_tx_obj, mp_nrf24_read_observe_tx);

STATIC mp_obj_t mp_nrf24_flush_rx(mp_obj_t self_in) {
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
    nrf24_flush_rx(&self->nrf);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(nrf24_flush_rx_obj, mp_nrf24_flush_rx);

STATIC mp_obj_t mp_nrf24_flush_tx(mp_obj_t self_in) {
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
    nrf24_flush_tx(&self->nrf);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(nrf24_flush_tx_obj, mp_nrf24_flush_tx);

STATIC mp_obj_t mp_nrf24_send_start(mp_obj_t self_in, mp_obj_t data_in) {
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_in, &bufinfo, MP_BUFFER_READ);
    if (bufinfo.len > self->nrf.payload_size) {
        mp_raise_ValueError(MP_ERROR_TEXT("payload length"));
    }
    if (!nrf24_send_start(&self->nrf, bufinfo.buf, bufinfo.len)) {
        mp_raise_OSError(MP_EBUSY);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(nrf24_send_start_obj, mp_nrf24_send_start);

STATIC mp_obj_t mp_nrf24_send_done(mp_obj_t self_in) {
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
    return MP_OBJ_NEW_SMALL_INT(nrf24_send_done(&self->nrf));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(nrf24_send_done_obj, mp_nrf24_send_done);

STATIC mp_obj_t mp_nrf24_abort_send(mp_obj_t self_in) {
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
    nrf24_abort_send(&self->nrf);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(nrf24_abort_send_obj, mp_nrf24_abort_send);

STATIC mp_obj_t mp_nrf24_restart_tx(mp_obj_t self_in) {
    mp_nrf24_obj_t *self = nrf24_obj_for_python(self_in);
    nrf24_restart_tx(&self->nrf);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(nrf24_restart_tx_obj, mp_nrf24_restart_tx);


STATIC const mp_rom_map_elem_t nrf24_locals[] = {
    // Core API & Power
    { MP_ROM_QSTR(MP_QSTR_power_up), MP_ROM_PTR(&nrf24_power_up_obj) },
    { MP_ROM_QSTR(MP_QSTR_power_down), MP_ROM_PTR(&nrf24_power_down_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_power_speed), MP_ROM_PTR(&nrf24_set_power_speed_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_channel), MP_ROM_PTR(&nrf24_set_channel_obj) },
    { MP_ROM_QSTR(MP_QSTR_open_tx_pipe), MP_ROM_PTR(&nrf24_open_tx_pipe_obj) },
    { MP_ROM_QSTR(MP_QSTR_open_rx_pipe), MP_ROM_PTR(&nrf24_open_rx_pipe_obj) },
    { MP_ROM_QSTR(MP_QSTR_close_rx_pipe), MP_ROM_PTR(&nrf24_close_rx_pipe_obj) },

    // RX / TX
    { MP_ROM_QSTR(MP_QSTR_start_listening), MP_ROM_PTR(&nrf24_start_listening_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop_listening), MP_ROM_PTR(&nrf24_stop_listening_obj) },
    { MP_ROM_QSTR(MP_QSTR_any), MP_ROM_PTR(&nrf24_any_obj) },
    { MP_ROM_QSTR(MP_QSTR_recv), MP_ROM_PTR(&nrf24_recv_obj) },
    { MP_ROM_QSTR(MP_QSTR_recv_into), MP_ROM_PTR(&nrf24_recv_into_obj) },
    { MP_ROM_QSTR(MP_QSTR_send), MP_ROM_PTR(&nrf24_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_send_start), MP_ROM_PTR(&nrf24_send_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_send_done), MP_ROM_PTR(&nrf24_send_done_obj) },
    { MP_ROM_QSTR(MP_QSTR_abort_send), MP_ROM_PTR(&nrf24_abort_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_restart_tx), MP_ROM_PTR(&nrf24_restart_tx_obj) },

    // Internal / Register functions
    { MP_ROM_QSTR(MP_QSTR_reg_read), MP_ROM_PTR(&nrf24_reg_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_reg_write), MP_ROM_PTR(&nrf24_reg_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_status), MP_ROM_PTR(&nrf24_read_status_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_irq_sources), MP_ROM_PTR(&nrf24_set_irq_sources_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_irq), MP_ROM_PTR(&nrf24_clear_irq_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_observe_tx), MP_ROM_PTR(&nrf24_read_observe_tx_obj) },
    { MP_ROM_QSTR(MP_QSTR_flush_rx), MP_ROM_PTR(&nrf24_flush_rx_obj) },
    { MP_ROM_QSTR(MP_QSTR_flush_tx), MP_ROM_PTR(&nrf24_flush_tx_obj) },

    // Constants (adjust values to match your nrf24.h definitions)
    { MP_ROM_QSTR(MP_QSTR_SPEED_250k), MP_ROM_INT(NRF24_SPEED_250K) },
    { MP_ROM_QSTR(MP_QSTR_SPEED_1M),   MP_ROM_INT(NRF24_SPEED_1M) },
    { MP_ROM_QSTR(MP_QSTR_SPEED_2M),   MP_ROM_INT(NRF24_SPEED_2M) },
    
    { MP_ROM_QSTR(MP_QSTR_POWER_0),    MP_ROM_INT(NRF24_POWER_0) },
    { MP_ROM_QSTR(MP_QSTR_POWER_1),    MP_ROM_INT(NRF24_POWER_1) },
    { MP_ROM_QSTR(MP_QSTR_POWER_2),    MP_ROM_INT(NRF24_POWER_2) },
    { MP_ROM_QSTR(MP_QSTR_POWER_3),    MP_ROM_INT(NRF24_POWER_3) },

    { MP_ROM_QSTR(MP_QSTR_IRQ_RX_DR),  MP_ROM_INT(NRF24_STATUS_RX_DR) },
    { MP_ROM_QSTR(MP_QSTR_IRQ_TX_DS),  MP_ROM_INT(NRF24_STATUS_TX_DS) },
    { MP_ROM_QSTR(MP_QSTR_IRQ_MAX_RT), MP_ROM_INT(NRF24_STATUS_MAX_RT) },
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
