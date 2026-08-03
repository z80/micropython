#include <string.h>

#include "py/obj.h"
#include "py/runtime.h"
#include "py/mphal.h"

#include "module_transport_core.h"
#include "module_transport_event.h"
#include "nrf24.h"
#include "nrf24_regs.h"
#include "transport_core.h"
#include "transport_stm32.h"

#ifndef STATIC
#define STATIC static
#endif

#ifndef MODULE_TRANSPORT_CORE_ENABLED
#define MODULE_TRANSPORT_CORE_ENABLED (1)
#endif

#if MODULE_TRANSPORT_CORE_ENABLED

#define MP_TRANSPORT_BUFFER_COMMAND (1)
#define MP_TRANSPORT_BUFFER_PIPE (2)
#define MP_TRANSPORT_BUFFER_CONTROL_TX (3)

#define MP_TRANSPORT_DEFAULT_COMMAND_SIZE (128)
#define MP_TRANSPORT_DEFAULT_PIPE_SIZE (2048)
#define MP_TRANSPORT_DEFAULT_EVENT_BYTES (512)

typedef struct _mp_transport_core_obj_t {
    mp_obj_base_t base;
    transport_core_t core;
    nrf24_t radio;
    transport_stm32_t hardware;
    mp_obj_t spi_obj;
    mp_obj_t cs_obj;
    mp_obj_t ce_obj;
    mp_obj_t irq_obj;
    mp_obj_t buffer_provider;
    mp_obj_t command_owners[TRANSPORT_CORE_COMMAND_SLOTS];
    mp_obj_t pipe_owners[TRANSPORT_CORE_PIPE_SLOTS];
    mp_obj_t control_tx_owner;
    bool closed;
} mp_transport_core_obj_t;

STATIC transport_stm32_t *transport_hardware_from_radio(nrf24_t *radio) {
    return (transport_stm32_t *)radio->user_data;
}

STATIC void transport_radio_spi_transfer(nrf24_t *radio, size_t length,
        const uint8_t *tx, uint8_t *rx) {
    (void)transport_stm32_spi_transfer(transport_hardware_from_radio(radio),
        length, tx, rx);
}

STATIC void transport_radio_cs_write(nrf24_t *radio, uint8_t state) {
    transport_stm32_cs_write(transport_hardware_from_radio(radio), state != 0);
}

STATIC void transport_radio_ce_write(nrf24_t *radio, uint8_t state) {
    transport_stm32_ce_write(transport_hardware_from_radio(radio), state != 0);
}

STATIC void transport_radio_delay_us(uint32_t delay_us) {
    mp_hal_delay_us(delay_us);
}

STATIC uint32_t transport_radio_ticks_ms(void) {
    return mp_hal_ticks_ms();
}

STATIC bool transport_core_irq_is_low(void *context) {
    return transport_stm32_irq_is_low((transport_stm32_t *)context);
}

STATIC bool transport_core_radio_io_ok(void *context) {
    return transport_stm32_get_error((transport_stm32_t *)context) ==
        TRANSPORT_STM32_ERROR_NONE;
}

STATIC void transport_core_radio_irq(void *context) {
    mp_transport_core_obj_t *self = context;
    transport_core_on_radio_irq(&self->core);
}

STATIC mp_transport_core_obj_t *transport_core_from_obj(mp_obj_t self_in) {
    if (!mp_obj_is_type(self_in, &mp_transport_core_type)) {
        mp_raise_TypeError(MP_ERROR_TEXT("Core required"));
    }
    mp_transport_core_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (self->closed) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("core closed"));
    }
    return self;
}

STATIC uint32_t transport_core_ticks_ms(void *context) {
    (void)context;
    return mp_hal_ticks_ms();
}

STATIC transport_critical_state_t transport_core_critical_enter(void *context) {
    (void)context;
    return (transport_critical_state_t)MICROPY_BEGIN_ATOMIC_SECTION();
}

STATIC void transport_core_critical_exit(void *context,
        transport_critical_state_t state) {
    (void)context;
    MICROPY_END_ATOMIC_SECTION((mp_uint_t)state);
}

STATIC mp_obj_t transport_core_request_buffer(mp_obj_t provider,
        uint8_t kind, uint8_t index, size_t minimum_size,
        uint8_t **buffer, size_t *capacity) {
    mp_obj_t args[3] = {
        MP_OBJ_NEW_SMALL_INT(kind),
        MP_OBJ_NEW_SMALL_INT(index),
        mp_obj_new_int_from_uint(minimum_size),
    };
    mp_obj_t owner = mp_call_function_n_kw(provider, 3, 0, args);
    mp_buffer_info_t info;
    mp_get_buffer_raise(owner, &info, MP_BUFFER_WRITE);
    if (info.len < minimum_size) {
        mp_raise_ValueError(MP_ERROR_TEXT("provided buffer too small"));
    }
    *buffer = info.buf;
    *capacity = info.len;
    return owner;
}

STATIC bool transport_regions_overlap(const uint8_t *a, size_t a_len,
        const uint8_t *b, size_t b_len) {
    uintptr_t a0 = (uintptr_t)a;
    uintptr_t b0 = (uintptr_t)b;
    uintptr_t a1 = a0 + a_len;
    uintptr_t b1 = b0 + b_len;
    if (a1 < a0 || b1 < b0) return true;
    return a0 < b1 && b0 < a1;
}

STATIC void transport_core_reject_overlapping_buffers(
        const mp_transport_core_obj_t *self) {
    const uint8_t *starts[TRANSPORT_CORE_COMMAND_SLOTS +
        TRANSPORT_CORE_PIPE_SLOTS + 1];
    size_t lengths[MP_ARRAY_SIZE(starts)];
    size_t count = 0;
    for (size_t i = 0; i < TRANSPORT_CORE_COMMAND_SLOTS; ++i) {
        starts[count] = self->core.commands[i].buffer.data;
        lengths[count++] = self->core.commands[i].buffer.capacity;
    }
    for (size_t i = 0; i < TRANSPORT_CORE_PIPE_SLOTS; ++i) {
        starts[count] = self->core.pipes[i].buffer.data;
        lengths[count++] = self->core.pipes[i].buffer.capacity;
    }
    starts[count] = self->core.control_tx.data;
    lengths[count++] = self->core.control_tx.capacity;
    for (size_t i = 0; i < count; ++i) {
        for (size_t j = i + 1; j < count; ++j) {
            if (transport_regions_overlap(starts[i], lengths[i],
                    starts[j], lengths[j])) {
                mp_raise_ValueError(MP_ERROR_TEXT("transport buffers overlap"));
            }
        }
    }
}

STATIC mp_obj_t transport_core_make_new(const mp_obj_type_t *type,
        size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum {
        ARG_spi, ARG_cs, ARG_ce, ARG_irq, ARG_buffer_provider,
        ARG_node_id, ARG_network_id, ARG_service_address,
        ARG_channel, ARG_data_rate, ARG_power,
        ARG_ard_us, ARG_arc,
        ARG_command_size, ARG_pipe_buffer_size,
        ARG_pipe_event_bytes, ARG_command_lease_ms, ARG_pipe_lease_ms,
        ARG_max_rt_window_ms, ARG_max_rt_restarts,
    };
    static const mp_arg_t allowed[] = {
        { MP_QSTR_spi, MP_ARG_REQUIRED | MP_ARG_OBJ, {} },
        { MP_QSTR_cs, MP_ARG_REQUIRED | MP_ARG_OBJ, {} },
        { MP_QSTR_ce, MP_ARG_REQUIRED | MP_ARG_OBJ, {} },
        { MP_QSTR_irq, MP_ARG_REQUIRED | MP_ARG_OBJ, {} },
        { MP_QSTR_buffer_provider, MP_ARG_REQUIRED | MP_ARG_OBJ, {} },
        { MP_QSTR_node_id, MP_ARG_INT, {.u_int = TRANSPORT_CORE_INVALID_ID} },
        { MP_QSTR_network_id, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_service_address, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_channel, MP_ARG_INT, {.u_int = 46} },
        { MP_QSTR_data_rate, MP_ARG_INT, {.u_int = NRF24_SPEED_2M} },
        { MP_QSTR_power, MP_ARG_INT, {.u_int = NRF24_POWER_3} },
        { MP_QSTR_ard_us, MP_ARG_INT, {.u_int = 1750} },
        { MP_QSTR_arc, MP_ARG_INT, {.u_int = 8} },
        { MP_QSTR_command_size, MP_ARG_INT,
          {.u_int = MP_TRANSPORT_DEFAULT_COMMAND_SIZE} },
        { MP_QSTR_pipe_buffer_size, MP_ARG_INT,
          {.u_int = MP_TRANSPORT_DEFAULT_PIPE_SIZE} },
        { MP_QSTR_pipe_event_bytes, MP_ARG_INT,
          {.u_int = MP_TRANSPORT_DEFAULT_EVENT_BYTES} },
        { MP_QSTR_command_lease_ms, MP_ARG_INT, {.u_int = 2000} },
        { MP_QSTR_pipe_lease_ms, MP_ARG_INT, {.u_int = 10000} },
        { MP_QSTR_max_rt_window_ms, MP_ARG_INT, {.u_int = 100} },
        { MP_QSTR_max_rt_restarts, MP_ARG_INT, {.u_int = -1} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args,
        MP_ARRAY_SIZE(allowed), allowed, args);

    if (args[ARG_node_id].u_int < 0 || args[ARG_node_id].u_int > 0xff
            || args[ARG_node_id].u_int == 0xfe
            || args[ARG_channel].u_int < 0 || args[ARG_channel].u_int > 125
            || (args[ARG_data_rate].u_int != NRF24_SPEED_250K &&
                args[ARG_data_rate].u_int != NRF24_SPEED_1M &&
                args[ARG_data_rate].u_int != NRF24_SPEED_2M)
            || args[ARG_power].u_int < NRF24_POWER_0
            || args[ARG_power].u_int > NRF24_POWER_3
            || (args[ARG_power].u_int & 1) != 0
            || args[ARG_ard_us].u_int < 250 || args[ARG_ard_us].u_int > 4000
            || args[ARG_arc].u_int < 0 || args[ARG_arc].u_int > 15
            || args[ARG_command_size].u_int <= 0
            || args[ARG_command_size].u_int > 0xffff
            || args[ARG_pipe_buffer_size].u_int <= 1
            || args[ARG_pipe_event_bytes].u_int <= 0
            || args[ARG_pipe_event_bytes].u_int > 0xffff
            || args[ARG_command_lease_ms].u_int < 0
            || args[ARG_command_lease_ms].u_int > 0xffff
            || args[ARG_pipe_lease_ms].u_int < 0
            || args[ARG_pipe_lease_ms].u_int > 0xffff
            || args[ARG_max_rt_window_ms].u_int < 0
            || args[ARG_max_rt_window_ms].u_int > 0xffff
            || args[ARG_max_rt_restarts].u_int < -1
            || args[ARG_max_rt_restarts].u_int >=
                (mp_int_t)TRANSPORT_CORE_RESTARTS_UNBOUNDED) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid core configuration"));
    }

    transport_core_config_t config;
    memset(&config, 0, sizeof(config));
    config.node_id = (uint8_t)args[ARG_node_id].u_int;
    config.network_id[0] = 0xd2;
    config.network_id[1] = 0x6a;
    config.network_id[2] = 0xb5;
    config.network_id[3] = 0x3c;

    if (args[ARG_network_id].u_obj != MP_OBJ_NULL &&
            args[ARG_network_id].u_obj != mp_const_none) {
        mp_buffer_info_t network;
        mp_get_buffer_raise(args[ARG_network_id].u_obj, &network,
            MP_BUFFER_READ);
        if (network.len != sizeof(config.network_id)) {
            mp_raise_ValueError(MP_ERROR_TEXT("network_id must be 4 bytes"));
        }
        memcpy(config.network_id, network.buf, sizeof(config.network_id));
    }

    if (args[ARG_service_address].u_obj != MP_OBJ_NULL &&
            args[ARG_service_address].u_obj != mp_const_none) {
        mp_buffer_info_t address;
        mp_get_buffer_raise(args[ARG_service_address].u_obj, &address,
            MP_BUFFER_READ);
        if (address.len != NRF24_ADDR_LEN) {
            mp_raise_ValueError(MP_ERROR_TEXT("service_address must be 5 bytes"));
        }
        memcpy(config.service_address, address.buf, NRF24_ADDR_LEN);
        config.has_service_address = true;
    }
    if (config.node_id == TRANSPORT_CORE_INVALID_ID &&
            !config.has_service_address) {
        mp_raise_ValueError(MP_ERROR_TEXT("unassigned node needs service_address"));
    }
    if (config.node_id == 0 && config.has_service_address &&
            (config.service_address[0] != TRANSPORT_CORE_INVALID_ID ||
             memcmp(config.service_address + 1, config.network_id,
                sizeof(config.network_id)) != 0)) {
        mp_raise_ValueError(MP_ERROR_TEXT("master service address suffix"));
    }

    config.preferred_pipe_event_bytes =
        (uint16_t)args[ARG_pipe_event_bytes].u_int;
    config.command_lease_ms = (uint16_t)args[ARG_command_lease_ms].u_int;
    config.pipe_lease_ms = (uint16_t)args[ARG_pipe_lease_ms].u_int;
    config.max_rt_window_ms = (uint16_t)args[ARG_max_rt_window_ms].u_int;
    config.max_rt_restarts = args[ARG_max_rt_restarts].u_int < 0 ?
        TRANSPORT_CORE_RESTARTS_UNBOUNDED :
        (uint8_t)args[ARG_max_rt_restarts].u_int;
    mp_transport_core_obj_t *self =
        mp_obj_malloc_with_finaliser(mp_transport_core_obj_t, type);
    memset(&self->core, 0, sizeof(self->core));
    memset(&self->radio, 0, sizeof(self->radio));
    memset(&self->hardware, 0, sizeof(self->hardware));
    self->spi_obj = args[ARG_spi].u_obj;
    self->cs_obj = args[ARG_cs].u_obj;
    self->ce_obj = args[ARG_ce].u_obj;
    self->irq_obj = args[ARG_irq].u_obj;
    self->buffer_provider = args[ARG_buffer_provider].u_obj;
    self->closed = true;
    for (size_t i = 0; i < TRANSPORT_CORE_COMMAND_SLOTS; ++i) {
        self->command_owners[i] = MP_OBJ_NULL;
    }
    for (size_t i = 0; i < TRANSPORT_CORE_PIPE_SLOTS; ++i) {
        self->pipe_owners[i] = MP_OBJ_NULL;
    }
    self->control_tx_owner = MP_OBJ_NULL;

    if (!transport_stm32_init(&self->hardware, self->spi_obj, self->cs_obj,
            self->ce_obj, self->irq_obj)) {
        switch (transport_stm32_get_error(&self->hardware)) {
            case TRANSPORT_STM32_ERROR_ARGUMENT:
                mp_raise_ValueError(
                    MP_ERROR_TEXT("transport hardware pins conflict"));
            case TRANSPORT_STM32_ERROR_SPI_CONFIGURATION:
                mp_raise_ValueError(
                    MP_ERROR_TEXT("unsupported SPI configuration"));
            case TRANSPORT_STM32_ERROR_SPI_NOT_READY:
                mp_raise_ValueError(MP_ERROR_TEXT("SPI not ready"));
            default:
                mp_raise_ValueError(
                    MP_ERROR_TEXT("invalid transport hardware"));
        }
    }
    self->radio.spi_transfer = transport_radio_spi_transfer;
    self->radio.csn_set = transport_radio_cs_write;
    self->radio.ce_set = transport_radio_ce_write;
    self->radio.delay_us = transport_radio_delay_us;
    self->radio.ticks_ms = transport_radio_ticks_ms;
    self->radio.user_data = &self->hardware;
    transport_stm32_clear_error(&self->hardware);
    if (!nrf24_init(&self->radio, (uint8_t)args[ARG_channel].u_int,
            NRF24_MAX_PAYLOAD, (uint8_t)args[ARG_data_rate].u_int,
            (uint8_t)args[ARG_power].u_int,
            (uint16_t)args[ARG_ard_us].u_int,
            (uint8_t)args[ARG_arc].u_int) ||
            !transport_core_radio_io_ok(&self->hardware)) {
        mp_raise_msg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("radio initialization failed"));
    }
    if (nrf24_reg_read(&self->radio, NRF24_REG_SETUP_AW) != 0x03 ||
            nrf24_reg_read(&self->radio, NRF24_REG_RF_CH) !=
                (uint8_t)args[ARG_channel].u_int ||
            !transport_core_radio_io_ok(&self->hardware)) {
        mp_raise_msg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("radio not responding"));
    }

    config.radio.radio = &self->radio;
    config.radio.ticks_ms = transport_core_ticks_ms;
    config.radio.irq_is_low = transport_core_irq_is_low;
    config.radio.io_ok = transport_core_radio_io_ok;
    config.radio.critical_enter = transport_core_critical_enter;
    config.radio.critical_exit = transport_core_critical_exit;
    config.radio.context = &self->hardware;
    transport_core_init(&self->core, &config);

    for (size_t i = 0; i < TRANSPORT_CORE_COMMAND_SLOTS; ++i) {
        uint8_t *buffer;
        size_t capacity;
        self->command_owners[i] = transport_core_request_buffer(
            self->buffer_provider, MP_TRANSPORT_BUFFER_COMMAND, i,
            (size_t)args[ARG_command_size].u_int, &buffer, &capacity);
        if (!transport_core_attach_command_buffer(
                &self->core, i, buffer, capacity)) {
            mp_raise_ValueError(MP_ERROR_TEXT("invalid command buffer"));
        }
    }
    for (size_t i = 0; i < TRANSPORT_CORE_PIPE_SLOTS; ++i) {
        uint8_t *buffer;
        size_t capacity;
        self->pipe_owners[i] = transport_core_request_buffer(
            self->buffer_provider, MP_TRANSPORT_BUFFER_PIPE, i,
            (size_t)args[ARG_pipe_buffer_size].u_int, &buffer, &capacity);
        if (!transport_core_attach_pipe_buffer(
                &self->core, i, buffer, capacity)) {
            mp_raise_ValueError(MP_ERROR_TEXT("invalid pipe buffer"));
        }
    }
    {
        uint8_t *buffer;
        size_t capacity;
        size_t minimum = TRANSPORT_CORE_CONTROL_TX_SLOTS * NRF24_MAX_PAYLOAD;
        self->control_tx_owner = transport_core_request_buffer(
            self->buffer_provider, MP_TRANSPORT_BUFFER_CONTROL_TX, 0,
            minimum, &buffer, &capacity);
        if (!transport_core_attach_control_tx_buffer(
                &self->core, buffer, capacity)) {
            mp_raise_ValueError(MP_ERROR_TEXT("invalid control TX buffer"));
        }
    }

    transport_core_reject_overlapping_buffers(self);
    self->closed = false;
    return MP_OBJ_FROM_PTR(self);
}

STATIC void transport_core_stop_hardware(mp_transport_core_obj_t *self) {
    transport_stm32_unregister_irq(&self->hardware);
    transport_stm32_ce_write(&self->hardware, false);
    if (self->hardware.initialized && !self->hardware.spi_busy) {
        transport_stm32_clear_error(&self->hardware);
        nrf24_stop_listening(&self->radio);
        nrf24_set_irq_sources(&self->radio, 0);
        nrf24_clear_irq(&self->radio, NRF24_STATUS_RX_DR |
            NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);
    }
    transport_core_stop(&self->core);
}

STATIC mp_obj_t mp_transport_core_start(mp_obj_t self_in) {
    mp_transport_core_obj_t *self = transport_core_from_obj(self_in);
    transport_stm32_clear_error(&self->hardware);
    if (!transport_core_start(&self->core)) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("core start failed"));
    }
    nrf24_set_irq_sources(&self->radio, NRF24_STATUS_RX_DR |
        NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);
    nrf24_clear_irq(&self->radio, NRF24_STATUS_RX_DR |
        NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);
    nrf24_start_listening(&self->radio);
    if (!transport_core_radio_io_ok(&self->hardware) ||
            !transport_stm32_register_irq(&self->hardware,
                transport_core_radio_irq, self)) {
        transport_core_stop_hardware(self);
        mp_raise_msg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("radio IRQ start failed"));
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(transport_core_start_obj,
    mp_transport_core_start);

STATIC mp_obj_t mp_transport_core_stop(mp_obj_t self_in) {
    mp_transport_core_obj_t *self = transport_core_from_obj(self_in);
    transport_core_stop_hardware(self);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(transport_core_stop_obj,
    mp_transport_core_stop);

STATIC mp_obj_t mp_transport_core_close(mp_obj_t self_in) {
    if (!mp_obj_is_type(self_in, &mp_transport_core_type)) {
        mp_raise_TypeError(MP_ERROR_TEXT("Core required"));
    }
    mp_transport_core_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (!self->closed) {
        transport_core_stop_hardware(self);
        transport_stm32_deinit(&self->hardware);
        self->closed = true;
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(transport_core_close_obj,
    mp_transport_core_close);

STATIC mp_obj_t mp_transport_core_del(mp_obj_t self_in) {
    if (mp_obj_is_type(self_in, &mp_transport_core_type)) {
        mp_transport_core_obj_t *self = MP_OBJ_TO_PTR(self_in);
        if (!self->closed) {
            transport_core_stop_hardware(self);
            transport_stm32_deinit(&self->hardware);
            self->closed = true;
        }
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(transport_core_del_obj,
    mp_transport_core_del);

STATIC mp_obj_t mp_transport_core_send_command(size_t n_args,
        const mp_obj_t *args) {
    mp_transport_core_obj_t *self = transport_core_from_obj(args[0]);
    mp_int_t destination = mp_obj_get_int(args[1]);
    mp_int_t message_type = mp_obj_get_int(args[2]);
    mp_int_t message_id = mp_obj_get_int(args[3]);
    mp_buffer_info_t data;
    size_t maximum = 0;
    if (!self->core.started) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("core not started"));
    }
    if (destination < 0 || destination >= TRANSPORT_CORE_INVALID_ID ||
            destination == self->core.config.node_id || message_id < 0 ||
            message_id > 0xff ||
            (message_type != TRANSPORT_WIRE_COMMAND &&
             message_type != TRANSPORT_WIRE_COMMAND_REPLY &&
             message_type != TRANSPORT_WIRE_MANAGEMENT_REQUEST &&
             message_type != TRANSPORT_WIRE_MANAGEMENT_REPLY)) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid command arguments"));
    }
    (void)n_args;
    mp_get_buffer_raise(args[4], &data, MP_BUFFER_READ);
    for (size_t i = 0; i < TRANSPORT_CORE_COMMAND_SLOTS; ++i) {
        if (self->core.commands[i].buffer.capacity > maximum) {
            maximum = self->core.commands[i].buffer.capacity;
        }
    }
    if (data.len > UINT16_MAX || data.len > maximum) {
        mp_raise_ValueError(MP_ERROR_TEXT("command too large"));
    }
    return mp_obj_new_bool(transport_core_send_command(&self->core,
        (uint8_t)destination, (uint8_t)message_type, (uint8_t)message_id,
        data.buf, data.len));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(transport_core_send_command_obj,
    5, 5, mp_transport_core_send_command);

STATIC mp_obj_t mp_transport_core_send_registration(size_t n_args,
        const mp_obj_t *args) {
    mp_transport_core_obj_t *self = transport_core_from_obj(args[0]);
    mp_int_t destination = mp_obj_get_int(args[1]);
    mp_int_t message_type = mp_obj_get_int(args[2]);
    mp_int_t message_id = mp_obj_get_int(args[3]);
    mp_buffer_info_t address;
    mp_buffer_info_t data;
    size_t expected_length;
    if (!self->core.started) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("core not started"));
    }
    if (destination < 0 || destination > 0xff || message_id < 0 ||
            message_id > 0xff ||
            (message_type != TRANSPORT_WIRE_ENUM_HELLO &&
             message_type != TRANSPORT_WIRE_ENUM_ASSIGN &&
             message_type != TRANSPORT_WIRE_ENUM_CONFIRM)) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid registration arguments"));
    }
    (void)n_args;
    mp_get_buffer_raise(args[4], &address, MP_BUFFER_READ);
    mp_get_buffer_raise(args[5], &data, MP_BUFFER_READ);
    expected_length = message_type == TRANSPORT_WIRE_ENUM_HELLO ? 8u : 9u;
    if (address.len != NRF24_ADDR_LEN || data.len != expected_length) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid registration buffers"));
    }
    return mp_obj_new_bool(transport_core_send_registration(&self->core,
        (uint8_t)message_type, (uint8_t)destination, (uint8_t)message_id,
        address.buf, data.buf, data.len));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    transport_core_send_registration_obj, 6, 6,
    mp_transport_core_send_registration);

STATIC mp_obj_t mp_transport_core_set_node_id(mp_obj_t self_in,
        mp_obj_t node_id_in) {
    mp_transport_core_obj_t *self = transport_core_from_obj(self_in);
    mp_int_t node_id = mp_obj_get_int(node_id_in);
    if (node_id < 0 || node_id > 0xff || node_id == 0xfe) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid node id"));
    }
    if (!transport_core_set_node_id(&self->core, (uint8_t)node_id)) {
        mp_raise_msg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("core busy or address failed"));
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(transport_core_set_node_id_obj,
    mp_transport_core_set_node_id);

STATIC mp_obj_t mp_transport_core_open_pipe(mp_obj_t self_in,
        mp_obj_t destination_in, mp_obj_t stream_id_in) {
    mp_transport_core_obj_t *self = transport_core_from_obj(self_in);
    mp_int_t destination = mp_obj_get_int(destination_in);
    mp_int_t stream_id = mp_obj_get_int(stream_id_in);
    int slot;
    if (!self->core.started) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("core not started"));
    }
    if (destination < 0 || destination >= TRANSPORT_CORE_INVALID_ID ||
            destination == self->core.config.node_id || stream_id < 0 ||
            stream_id > 0xff) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid pipe arguments"));
    }
    slot = transport_core_open_pipe(&self->core, (uint8_t)destination,
        (uint8_t)stream_id);
    return slot < 0 ? mp_const_none : MP_OBJ_NEW_SMALL_INT(slot);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(transport_core_open_pipe_obj,
    mp_transport_core_open_pipe);

STATIC mp_obj_t mp_transport_core_pipe_write(mp_obj_t self_in,
        mp_obj_t slot_in, mp_obj_t data_in) {
    mp_transport_core_obj_t *self = transport_core_from_obj(self_in);
    mp_int_t slot = mp_obj_get_int(slot_in);
    mp_buffer_info_t data;
    transport_pipe_slot_t *pipe;
    if (slot < 0 || slot >= TRANSPORT_CORE_PIPE_SLOTS) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid pipe slot"));
    }
    pipe = &self->core.pipes[slot];
    if (pipe->direction != TRANSPORT_DIRECTION_TX ||
            (pipe->state != TRANSPORT_PIPE_TX_INTENT_QUEUED &&
             pipe->state != TRANSPORT_PIPE_TX_WAIT_CTS &&
             pipe->state != TRANSPORT_PIPE_TX_OPEN)) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("pipe not writable"));
    }
    mp_get_buffer_raise(data_in, &data, MP_BUFFER_READ);
    return mp_obj_new_int_from_uint(transport_core_pipe_tx_write(&self->core,
        (uint8_t)slot, data.buf, data.len));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(transport_core_pipe_write_obj,
    mp_transport_core_pipe_write);

STATIC mp_obj_t mp_transport_core_close_pipe(mp_obj_t self_in,
        mp_obj_t slot_in) {
    mp_transport_core_obj_t *self = transport_core_from_obj(self_in);
    mp_int_t slot = mp_obj_get_int(slot_in);
    if (slot < 0 || slot >= TRANSPORT_CORE_PIPE_SLOTS) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid pipe slot"));
    }
    if (!transport_core_close_pipe(&self->core, (uint8_t)slot)) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("pipe not open"));
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(transport_core_close_pipe_obj,
    mp_transport_core_close_pipe);

STATIC mp_obj_t mp_transport_core_poll_into(mp_obj_t self_in,
        mp_obj_t event_in) {
    mp_transport_core_obj_t *self = transport_core_from_obj(self_in);
    uint8_t *buffer;
    size_t capacity;
    size_t event_length = 0;
    size_t required_capacity = 0;
    mp_transport_event_get_buffer(event_in, &buffer, &capacity);
    mp_transport_event_invalidate(event_in);
    transport_poll_result_t result = transport_core_poll_into(
        &self->core, buffer, capacity, &event_length, &required_capacity);
    if (result == TRANSPORT_POLL_EVENT) {
        mp_transport_event_set_length(event_in, event_length);
        return mp_const_true;
    }
    if (result == TRANSPORT_POLL_EMPTY) {
        return mp_const_false;
    }
    if (result == TRANSPORT_POLL_BUFFER_TOO_SMALL) {
        mp_raise_ValueError(MP_ERROR_TEXT("event buffer too small"));
    }
    mp_raise_ValueError(MP_ERROR_TEXT("invalid poll arguments"));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(transport_core_poll_into_obj,
    mp_transport_core_poll_into);

STATIC mp_obj_t mp_transport_core_recommended_event_size(mp_obj_t self_in) {
    mp_transport_core_obj_t *self = transport_core_from_obj(self_in);
    size_t size = TRANSPORT_EVENT_HEADER_SIZE
        + self->core.config.preferred_pipe_event_bytes;
    for (size_t i = 0; i < TRANSPORT_CORE_COMMAND_SLOTS; ++i) {
        size_t command_size = TRANSPORT_EVENT_HEADER_SIZE
            + self->core.commands[i].buffer.capacity;
        if (command_size > size) {
            size = command_size;
        }
    }
    return mp_obj_new_int_from_uint(size);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(transport_core_recommended_event_size_obj,
    mp_transport_core_recommended_event_size);

STATIC mp_obj_t mp_transport_core_sticky_errors(mp_obj_t self_in) {
    mp_transport_core_obj_t *self = transport_core_from_obj(self_in);
    return mp_obj_new_int_from_uint(
        transport_core_get_sticky_errors(&self->core));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(transport_core_sticky_errors_obj,
    mp_transport_core_sticky_errors);

STATIC mp_obj_t mp_transport_core_clear_errors(size_t n_args,
        const mp_obj_t *args) {
    mp_transport_core_obj_t *self = transport_core_from_obj(args[0]);
    uint32_t mask = 0xffffffffu;
    if (n_args == 2) {
        mask = (uint32_t)mp_obj_get_int_truncated(args[1]);
    }
    transport_core_clear_sticky_errors(&self->core, mask);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(transport_core_clear_errors_obj,
    1, 2, mp_transport_core_clear_errors);

STATIC mp_obj_t mp_transport_core_stats(mp_obj_t self_in) {
    mp_transport_core_obj_t *self = transport_core_from_obj(self_in);
    const transport_core_stats_t *stats =
        transport_core_get_stats(&self->core);
    mp_obj_t values[23] = {
        mp_obj_new_int_from_uint(stats->radio_irqs),
        mp_obj_new_int_from_uint(stats->events_emitted),
        mp_obj_new_int_from_uint(stats->events_polled),
        mp_obj_new_int_from_uint(stats->rx_packets),
        mp_obj_new_int_from_uint(stats->tx_packets),
        mp_obj_new_int_from_uint(stats->event_queue_overflows),
        mp_obj_new_int_from_uint(stats->rx_overruns),
        mp_obj_new_int_from_uint(stats->tx_underruns),
        mp_obj_new_int_from_uint(stats->max_rt_events),
        mp_obj_new_int_from_uint(stats->max_rt_restarts),
        mp_obj_new_int_from_uint(stats->protocol_errors),
        mp_obj_new_int_from_uint(stats->pipe_rx_bytes),
        mp_obj_new_int_from_uint(stats->pipe_tx_bytes),
        mp_obj_new_int_from_uint(stats->control_packets_queued),
        mp_obj_new_int_from_uint(stats->control_packets_acked),
        mp_obj_new_int_from_uint(stats->commands_sent),
        mp_obj_new_int_from_uint(stats->commands_failed),
        mp_obj_new_int_from_uint(stats->pipes_opened),
        mp_obj_new_int_from_uint(stats->pipes_closed),
        mp_obj_new_int_from_uint(stats->pipes_failed),
        mp_obj_new_int_from_uint(stats->registration_rx),
        mp_obj_new_int_from_uint(stats->registration_sent),
        mp_obj_new_int_from_uint(stats->registration_failed),
    };
    return mp_obj_new_tuple(MP_ARRAY_SIZE(values), values);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(transport_core_stats_obj,
    mp_transport_core_stats);

STATIC const mp_rom_map_elem_t transport_core_locals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___del__), MP_ROM_PTR(&transport_core_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_start), MP_ROM_PTR(&transport_core_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop), MP_ROM_PTR(&transport_core_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_close), MP_ROM_PTR(&transport_core_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_send_command),
      MP_ROM_PTR(&transport_core_send_command_obj) },
    { MP_ROM_QSTR(MP_QSTR_send_registration),
      MP_ROM_PTR(&transport_core_send_registration_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_node_id),
      MP_ROM_PTR(&transport_core_set_node_id_obj) },
    { MP_ROM_QSTR(MP_QSTR_open_pipe),
      MP_ROM_PTR(&transport_core_open_pipe_obj) },
    { MP_ROM_QSTR(MP_QSTR_pipe_write),
      MP_ROM_PTR(&transport_core_pipe_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_close_pipe),
      MP_ROM_PTR(&transport_core_close_pipe_obj) },
    { MP_ROM_QSTR(MP_QSTR_poll_into),
      MP_ROM_PTR(&transport_core_poll_into_obj) },
    { MP_ROM_QSTR(MP_QSTR_recommended_event_size),
      MP_ROM_PTR(&transport_core_recommended_event_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_sticky_errors),
      MP_ROM_PTR(&transport_core_sticky_errors_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_errors),
      MP_ROM_PTR(&transport_core_clear_errors_obj) },
    { MP_ROM_QSTR(MP_QSTR_stats), MP_ROM_PTR(&transport_core_stats_obj) },
};
STATIC MP_DEFINE_CONST_DICT(transport_core_locals_dict,
    transport_core_locals_table);

MP_DEFINE_CONST_OBJ_TYPE(
    mp_transport_core_type,
    MP_QSTR_Core,
    MP_TYPE_FLAG_NONE,
    make_new, transport_core_make_new,
    locals_dict, &transport_core_locals_dict
);

STATIC const mp_rom_map_elem_t transport_core_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_transport_core) },
    { MP_ROM_QSTR(MP_QSTR_Core), MP_ROM_PTR(&mp_transport_core_type) },
    { MP_ROM_QSTR(MP_QSTR_Event), MP_ROM_PTR(&mp_transport_event_type) },
    { MP_ROM_QSTR(MP_QSTR_BUFFER_COMMAND),
      MP_ROM_INT(MP_TRANSPORT_BUFFER_COMMAND) },
    { MP_ROM_QSTR(MP_QSTR_BUFFER_PIPE),
      MP_ROM_INT(MP_TRANSPORT_BUFFER_PIPE) },
    { MP_ROM_QSTR(MP_QSTR_BUFFER_CONTROL_TX),
      MP_ROM_INT(MP_TRANSPORT_BUFFER_CONTROL_TX) },
    { MP_ROM_QSTR(MP_QSTR_COMMAND_SLOTS),
      MP_ROM_INT(TRANSPORT_CORE_COMMAND_SLOTS) },
    { MP_ROM_QSTR(MP_QSTR_PIPE_SLOTS),
      MP_ROM_INT(TRANSPORT_CORE_PIPE_SLOTS) },
    { MP_ROM_QSTR(MP_QSTR_CONTROL_TX_SLOTS),
      MP_ROM_INT(TRANSPORT_CORE_CONTROL_TX_SLOTS) },
    { MP_ROM_QSTR(MP_QSTR_INVALID_ID),
      MP_ROM_INT(TRANSPORT_CORE_INVALID_ID) },
    { MP_ROM_QSTR(MP_QSTR_MAX_COMMAND_LENGTH), MP_ROM_INT(0xffff) },
    { MP_ROM_QSTR(MP_QSTR_WIRE_MAX_DATA),
      MP_ROM_INT(TRANSPORT_WIRE_MAX_DATA) },
    { MP_ROM_QSTR(MP_QSTR_REGISTRATION_MAX_DATA),
      MP_ROM_INT(TRANSPORT_REGISTRATION_MAX_DATA) },
    { MP_ROM_QSTR(MP_QSTR_TYPE_COMMAND),
      MP_ROM_INT(TRANSPORT_WIRE_COMMAND) },
    { MP_ROM_QSTR(MP_QSTR_TYPE_COMMAND_REPLY),
      MP_ROM_INT(TRANSPORT_WIRE_COMMAND_REPLY) },
    { MP_ROM_QSTR(MP_QSTR_TYPE_MANAGEMENT_REQUEST),
      MP_ROM_INT(TRANSPORT_WIRE_MANAGEMENT_REQUEST) },
    { MP_ROM_QSTR(MP_QSTR_TYPE_MANAGEMENT_REPLY),
      MP_ROM_INT(TRANSPORT_WIRE_MANAGEMENT_REPLY) },
    { MP_ROM_QSTR(MP_QSTR_TYPE_STREAM),
      MP_ROM_INT(TRANSPORT_WIRE_STREAM) },
    { MP_ROM_QSTR(MP_QSTR_TYPE_ENUM_HELLO),
      MP_ROM_INT(TRANSPORT_WIRE_ENUM_HELLO) },
    { MP_ROM_QSTR(MP_QSTR_TYPE_ENUM_ASSIGN),
      MP_ROM_INT(TRANSPORT_WIRE_ENUM_ASSIGN) },
    { MP_ROM_QSTR(MP_QSTR_TYPE_ENUM_CONFIRM),
      MP_ROM_INT(TRANSPORT_WIRE_ENUM_CONFIRM) },
    { MP_ROM_QSTR(MP_QSTR_REGISTRATION_FAILURE_RADIO),
      MP_ROM_INT(TRANSPORT_REGISTRATION_FAILURE_RADIO) },
    { MP_ROM_QSTR(MP_QSTR_COMMAND_FAILURE_TIMEOUT),
      MP_ROM_INT(TRANSPORT_COMMAND_FAILURE_TIMEOUT) },
    { MP_ROM_QSTR(MP_QSTR_COMMAND_FAILURE_RADIO),
      MP_ROM_INT(TRANSPORT_COMMAND_FAILURE_RADIO) },
    { MP_ROM_QSTR(MP_QSTR_COMMAND_FAILURE_CTS_BASE),
      MP_ROM_INT(TRANSPORT_COMMAND_FAILURE_CTS_BASE) },
    { MP_ROM_QSTR(MP_QSTR_COMMAND_FAILURE_CTS_NO_BUFFER),
      MP_ROM_INT(TRANSPORT_COMMAND_FAILURE_CTS_BASE +
          TRANSPORT_CTS_NO_BUFFER) },
    { MP_ROM_QSTR(MP_QSTR_COMMAND_FAILURE_CTS_TOO_LARGE),
      MP_ROM_INT(TRANSPORT_COMMAND_FAILURE_CTS_BASE +
          TRANSPORT_CTS_TOO_LARGE) },
    { MP_ROM_QSTR(MP_QSTR_COMMAND_FAILURE_CTS_BUSY),
      MP_ROM_INT(TRANSPORT_COMMAND_FAILURE_CTS_BASE + TRANSPORT_CTS_BUSY) },
    { MP_ROM_QSTR(MP_QSTR_COMMAND_FAILURE_CTS_INVALID),
      MP_ROM_INT(TRANSPORT_COMMAND_FAILURE_CTS_BASE +
          TRANSPORT_CTS_INVALID) },
    { MP_ROM_QSTR(MP_QSTR_PIPE_FAILURE_TIMEOUT),
      MP_ROM_INT(TRANSPORT_PIPE_FAILURE_TIMEOUT) },
    { MP_ROM_QSTR(MP_QSTR_PIPE_FAILURE_RADIO),
      MP_ROM_INT(TRANSPORT_PIPE_FAILURE_RADIO) },
    { MP_ROM_QSTR(MP_QSTR_PIPE_FAILURE_CTS_BASE),
      MP_ROM_INT(TRANSPORT_PIPE_FAILURE_CTS_BASE) },
    { MP_ROM_QSTR(MP_QSTR_PIPE_FAILURE_CTS_NO_BUFFER),
      MP_ROM_INT(TRANSPORT_PIPE_FAILURE_CTS_BASE + TRANSPORT_CTS_NO_BUFFER) },
    { MP_ROM_QSTR(MP_QSTR_PIPE_FAILURE_CTS_BUSY),
      MP_ROM_INT(TRANSPORT_PIPE_FAILURE_CTS_BASE + TRANSPORT_CTS_BUSY) },
    { MP_ROM_QSTR(MP_QSTR_PIPE_FAILURE_CTS_INVALID),
      MP_ROM_INT(TRANSPORT_PIPE_FAILURE_CTS_BASE + TRANSPORT_CTS_INVALID) },
    { MP_ROM_QSTR(MP_QSTR_EVENT_FLAG_TX),
      MP_ROM_INT(TRANSPORT_EVENT_FLAG_TX) },
    { MP_ROM_QSTR(MP_QSTR_EVENT_NONE), MP_ROM_INT(TRANSPORT_EVENT_NONE) },
    { MP_ROM_QSTR(MP_QSTR_EVENT_COMMAND_READY),
      MP_ROM_INT(TRANSPORT_EVENT_COMMAND_READY) },
    { MP_ROM_QSTR(MP_QSTR_EVENT_COMMAND_SENT),
      MP_ROM_INT(TRANSPORT_EVENT_COMMAND_SENT) },
    { MP_ROM_QSTR(MP_QSTR_EVENT_COMMAND_FAILED),
      MP_ROM_INT(TRANSPORT_EVENT_COMMAND_FAILED) },
    { MP_ROM_QSTR(MP_QSTR_EVENT_PIPE_OPENED),
      MP_ROM_INT(TRANSPORT_EVENT_PIPE_OPENED) },
    { MP_ROM_QSTR(MP_QSTR_EVENT_PIPE_RX_DATA),
      MP_ROM_INT(TRANSPORT_EVENT_PIPE_RX_DATA) },
    { MP_ROM_QSTR(MP_QSTR_EVENT_PIPE_TX_SPACE),
      MP_ROM_INT(TRANSPORT_EVENT_PIPE_TX_SPACE) },
    { MP_ROM_QSTR(MP_QSTR_EVENT_PIPE_CLOSED),
      MP_ROM_INT(TRANSPORT_EVENT_PIPE_CLOSED) },
    { MP_ROM_QSTR(MP_QSTR_EVENT_PIPE_FAILED),
      MP_ROM_INT(TRANSPORT_EVENT_PIPE_FAILED) },
    { MP_ROM_QSTR(MP_QSTR_EVENT_NODE_FAILED),
      MP_ROM_INT(TRANSPORT_EVENT_NODE_FAILED) },
    { MP_ROM_QSTR(MP_QSTR_EVENT_CORE_ERROR),
      MP_ROM_INT(TRANSPORT_EVENT_CORE_ERROR) },
    { MP_ROM_QSTR(MP_QSTR_EVENT_REGISTRATION_RECEIVED),
      MP_ROM_INT(TRANSPORT_EVENT_REGISTRATION_RECEIVED) },
    { MP_ROM_QSTR(MP_QSTR_EVENT_REGISTRATION_SENT),
      MP_ROM_INT(TRANSPORT_EVENT_REGISTRATION_SENT) },
    { MP_ROM_QSTR(MP_QSTR_EVENT_REGISTRATION_FAILED),
      MP_ROM_INT(TRANSPORT_EVENT_REGISTRATION_FAILED) },
};
STATIC MP_DEFINE_CONST_DICT(transport_core_module_globals,
    transport_core_module_globals_table);

const mp_obj_module_t transport_core_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&transport_core_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_transport_core, transport_core_user_cmodule);

#endif /* MODULE_TRANSPORT_CORE_ENABLED */
