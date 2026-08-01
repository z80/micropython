#include "py/obj.h"
#include "py/objarray.h"
#include "py/runtime.h"

#include "module_transport_event.h"
#include "transport_event.h"

#ifndef STATIC
#define STATIC static
#endif

STATIC mp_transport_event_obj_t *transport_event_from_obj(mp_obj_t event_in) {
    if (!mp_obj_is_type(event_in, &mp_transport_event_type)) {
        mp_raise_TypeError(MP_ERROR_TEXT("Event required"));
    }
    return MP_OBJ_TO_PTR(event_in);
}

STATIC void transport_event_require_valid(mp_transport_event_obj_t *self) {
    if (!self->valid) {
        mp_raise_RuntimeError(MP_ERROR_TEXT("event not valid"));
    }
}

STATIC const uint8_t *transport_event_checked_data(
    mp_transport_event_obj_t *self, size_t *data_length) {
    transport_event_require_valid(self);

    const uint8_t *data;
    if (!transport_event_data(self->buffer, self->length, &data,
        data_length)) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid event record"));
    }
    return data;
}

mp_transport_event_obj_t *mp_transport_event_get(mp_obj_t event_in) {
    return transport_event_from_obj(event_in);
}

void mp_transport_event_get_buffer(mp_obj_t event_in, uint8_t **buffer,
    size_t *capacity) {
    mp_transport_event_obj_t *self = transport_event_from_obj(event_in);
    *buffer = self->buffer;
    *capacity = self->capacity;
}

void mp_transport_event_invalidate(mp_obj_t event_in) {
    mp_transport_event_obj_t *self = transport_event_from_obj(event_in);
    self->length = 0;
    self->valid = false;
}

void mp_transport_event_set_length(mp_obj_t event_in, size_t length) {
    mp_transport_event_obj_t *self = transport_event_from_obj(event_in);

    self->length = 0;
    self->valid = false;
    if (length < TRANSPORT_EVENT_HEADER_SIZE || length > self->capacity) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid event length"));
    }

    const uint8_t *data;
    size_t data_length;
    if (!transport_event_data(self->buffer, length, &data, &data_length)) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid event payload"));
    }

    self->length = length;
    self->valid = true;
}

STATIC mp_obj_t transport_event_make_new(const mp_obj_type_t *type,
    size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    mp_arg_check_num(n_args, n_kw, 1, 1, false);

    mp_obj_t owner;
    if (mp_obj_is_int(all_args[0])) {
        mp_int_t requested = mp_obj_get_int(all_args[0]);
        if (requested < TRANSPORT_EVENT_HEADER_SIZE) {
            mp_raise_ValueError(MP_ERROR_TEXT("event buffer too small"));
        }
        owner = mp_obj_new_bytearray((size_t)requested, NULL);
    } else {
        owner = all_args[0];
    }

    mp_buffer_info_t buffer_info;
    mp_get_buffer_raise(owner, &buffer_info, MP_BUFFER_WRITE);
    if (buffer_info.len < TRANSPORT_EVENT_HEADER_SIZE) {
        mp_raise_ValueError(MP_ERROR_TEXT("event buffer too small"));
    }

    mp_transport_event_obj_t *self =
        mp_obj_malloc(mp_transport_event_obj_t, type);
    self->buffer_owner = owner;
    self->buffer = buffer_info.buf;
    self->capacity = buffer_info.len;
    self->length = 0;
    self->valid = false;
    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t mp_transport_event_capacity(mp_obj_t self_in) {
    mp_transport_event_obj_t *self = transport_event_from_obj(self_in);
    return mp_obj_new_int_from_uint(self->capacity);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(transport_event_capacity_obj,
    mp_transport_event_capacity);

STATIC mp_obj_t mp_transport_event_length(mp_obj_t self_in) {
    mp_transport_event_obj_t *self = transport_event_from_obj(self_in);
    return mp_obj_new_int_from_uint(self->valid ? self->length : 0);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(transport_event_length_obj,
    mp_transport_event_length);

STATIC mp_obj_t mp_transport_event_valid(mp_obj_t self_in) {
    mp_transport_event_obj_t *self = transport_event_from_obj(self_in);
    return mp_obj_new_bool(self->valid);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(transport_event_valid_obj,
    mp_transport_event_valid);

#define DEFINE_EVENT_GETTER(py_name, c_getter, c_type) \
    STATIC mp_obj_t mp_transport_event_get_##py_name(mp_obj_t self_in) { \
        mp_transport_event_obj_t *self = transport_event_from_obj(self_in); \
        transport_event_require_valid(self); \
        c_type value; \
        if (!c_getter(self->buffer, self->length, &value)) { \
            mp_raise_ValueError(MP_ERROR_TEXT("invalid event record")); \
        } \
        return mp_obj_new_int_from_uint(value); \
    } \
    STATIC MP_DEFINE_CONST_FUN_OBJ_1(transport_event_##py_name##_obj, \
        mp_transport_event_get_##py_name)

DEFINE_EVENT_GETTER(type, transport_event_type, uint8_t);
DEFINE_EVENT_GETTER(flags, transport_event_flags, uint8_t);
DEFINE_EVENT_GETTER(object_id, transport_event_object_id, uint8_t);
DEFINE_EVENT_GETTER(source, transport_event_source, uint8_t);
DEFINE_EVENT_GETTER(transaction, transport_event_transaction, uint16_t);
DEFINE_EVENT_GETTER(data_length, transport_event_data_length, uint16_t);
DEFINE_EVENT_GETTER(value0, transport_event_value0, uint32_t);
DEFINE_EVENT_GETTER(value1, transport_event_value1, uint32_t);

STATIC mp_obj_t mp_transport_event_data(mp_obj_t self_in) {
    mp_transport_event_obj_t *self = transport_event_from_obj(self_in);
    size_t data_length;
    (void)transport_event_checked_data(self, &data_length);

    /* Event itself exports the payload through the buffer protocol. Returning
     * it keeps both Event and its backing owner rooted; memoryview(event.data())
     * is therefore zero-copy and cannot dangle after Event collection. */
    return self_in;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(transport_event_data_obj,
    mp_transport_event_data);

STATIC const mp_rom_map_elem_t transport_event_locals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_capacity), MP_ROM_PTR(&transport_event_capacity_obj) },
    { MP_ROM_QSTR(MP_QSTR_length), MP_ROM_PTR(&transport_event_length_obj) },
    { MP_ROM_QSTR(MP_QSTR_valid), MP_ROM_PTR(&transport_event_valid_obj) },
    { MP_ROM_QSTR(MP_QSTR_type), MP_ROM_PTR(&transport_event_type_obj) },
    { MP_ROM_QSTR(MP_QSTR_flags), MP_ROM_PTR(&transport_event_flags_obj) },
    { MP_ROM_QSTR(MP_QSTR_object_id), MP_ROM_PTR(&transport_event_object_id_obj) },
    { MP_ROM_QSTR(MP_QSTR_source), MP_ROM_PTR(&transport_event_source_obj) },
    { MP_ROM_QSTR(MP_QSTR_transaction), MP_ROM_PTR(&transport_event_transaction_obj) },
    { MP_ROM_QSTR(MP_QSTR_data_length), MP_ROM_PTR(&transport_event_data_length_obj) },
    { MP_ROM_QSTR(MP_QSTR_value0), MP_ROM_PTR(&transport_event_value0_obj) },
    { MP_ROM_QSTR(MP_QSTR_value1), MP_ROM_PTR(&transport_event_value1_obj) },
    { MP_ROM_QSTR(MP_QSTR_data), MP_ROM_PTR(&transport_event_data_obj) },
};
STATIC MP_DEFINE_CONST_DICT(transport_event_locals_dict,
    transport_event_locals_table);

STATIC mp_int_t transport_event_get_buffer(mp_obj_t self_in,
        mp_buffer_info_t *bufinfo, mp_uint_t flags) {
    mp_transport_event_obj_t *self = transport_event_from_obj(self_in);
    size_t data_length;
    const uint8_t *data = transport_event_checked_data(self, &data_length);
    if ((flags & MP_BUFFER_WRITE) != 0) return 1;
    bufinfo->buf = (void *)data;
    bufinfo->len = data_length;
    bufinfo->typecode = 'B';
    return 0;
}

MP_DEFINE_CONST_OBJ_TYPE(
    mp_transport_event_type,
    MP_QSTR_Event,
    MP_TYPE_FLAG_NONE,
    make_new, transport_event_make_new,
    buffer, transport_event_get_buffer,
    locals_dict, &transport_event_locals_dict
);
