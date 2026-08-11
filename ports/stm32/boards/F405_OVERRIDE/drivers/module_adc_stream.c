#include <stdint.h>
#include <string.h>

#include "py/runtime.h"
#include "py/objarray.h"
#include "pin.h"
#include "timer.h"
#include "adc_stream_stm32.h"

typedef struct _mp_adc_stream_obj_t {
    mp_obj_base_t base;
    mp_obj_t buffer_obj;
    mp_obj_t timer_obj;
    mp_obj_t half_view[2];
    const machine_pin_obj_t *pin;
    size_t half_samples;
    uint16_t acquisition_cycles;
    adc_stream_hw_t hw;
} mp_adc_stream_obj_t;

static const mp_obj_type_t mp_adc_stream_type;

static mp_adc_stream_obj_t *adc_stream_from_obj(mp_obj_t self_in) {
    if (!mp_obj_is_type(self_in, &mp_adc_stream_type)) {
        mp_raise_TypeError(MP_ERROR_TEXT("ADCStream required"));
    }
    return MP_OBJ_TO_PTR(self_in);
}

static mp_obj_t adc_stream_make_new(const mp_obj_type_t *type,
    size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 3, 3, false);

    const machine_pin_obj_t *pin = pin_find(args[0]);
    if ((pin->adc_num & PIN_ADC1) == 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("pin has no ADC1 channel"));
    }

    mp_buffer_info_t buffer;
    mp_get_buffer_raise(args[1], &buffer, MP_BUFFER_WRITE);
    if (!mp_obj_is_type(args[1], &mp_type_array)
        || (buffer.typecode & 0x7f) != 'H') {
        mp_raise_TypeError(MP_ERROR_TEXT("buffer must be array('H')"));
    }
    if (((uintptr_t)buffer.buf & 1) != 0 || buffer.len < 4
        || (buffer.len & 3) != 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("buffer needs an even sample count"));
    }
    size_t sample_count = buffer.len / sizeof(uint16_t);
    if (sample_count > UINT16_MAX) {
        mp_raise_ValueError(MP_ERROR_TEXT("buffer has more than 65535 samples"));
    }

    TIM_HandleTypeDef *timer = pyb_timer_get_handle(args[2]);
    if (timer->State == HAL_TIM_STATE_RESET
        || (timer->Instance != TIM2 && timer->Instance != TIM3
            && timer->Instance != TIM8)) {
        mp_raise_ValueError(MP_ERROR_TEXT("use initialized Timer(2), Timer(3), or Timer(8)"));
    }

    mp_adc_stream_obj_t *self =
        mp_obj_malloc_with_finaliser(mp_adc_stream_obj_t, type);
    memset(&self->hw, 0, sizeof(self->hw));
    self->buffer_obj = args[1];
    self->timer_obj = args[2];
    self->pin = pin;
    self->half_samples = sample_count / 2;
    self->acquisition_cycles = 56;

    uint8_t view_type = MP_OBJ_ARRAY_TYPECODE_FLAG_RW | 'H';
    self->half_view[0] = mp_obj_new_memoryview(view_type,
        self->half_samples, buffer.buf);
    self->half_view[1] = mp_obj_new_memoryview(view_type,
        self->half_samples, buffer.buf);
    mp_obj_array_t *second = MP_OBJ_TO_PTR(self->half_view[1]);
    second->free = self->half_samples;

    return MP_OBJ_FROM_PTR(self);
}

static mp_obj_t adc_stream_start(mp_obj_t self_in) {
    mp_adc_stream_obj_t *self = adc_stream_from_obj(self_in);
    if (adc_stream_hw_running(&self->hw)) {
        return mp_const_none;
    }

    mp_buffer_info_t buffer;
    mp_get_buffer_raise(self->buffer_obj, &buffer, MP_BUFFER_WRITE);
    size_t expected_bytes = self->half_samples * 2 * sizeof(uint16_t);
    mp_obj_array_t *first = MP_OBJ_TO_PTR(self->half_view[0]);
    if (buffer.len != expected_bytes || buffer.buf != first->items) {
        mp_raise_msg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("ADC buffer was resized"));
    }

    mp_hal_pin_config(self->pin, MP_HAL_PIN_MODE_ADC,
        MP_HAL_PIN_PULL_NONE, 0);
    TIM_HandleTypeDef *timer = pyb_timer_get_handle(self->timer_obj);
    if (!adc_stream_hw_start(&self->hw, self->pin->adc_channel, timer,
        self->acquisition_cycles, buffer.buf,
        buffer.len / sizeof(uint16_t))) {
        mp_raise_msg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("ADC stream start failed"));
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(adc_stream_start_obj, adc_stream_start);

static mp_obj_t adc_stream_stop(mp_obj_t self_in) {
    mp_adc_stream_obj_t *self = adc_stream_from_obj(self_in);
    adc_stream_hw_stop(&self->hw);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(adc_stream_stop_obj, adc_stream_stop);

static mp_obj_t adc_stream_poll(mp_obj_t self_in) {
    mp_adc_stream_obj_t *self = adc_stream_from_obj(self_in);
    int half = adc_stream_hw_take_ready(&self->hw);
    return half < 0 ? mp_const_none : self->half_view[half];
}
static MP_DEFINE_CONST_FUN_OBJ_1(adc_stream_poll_obj, adc_stream_poll);

static mp_obj_t adc_stream_available(mp_obj_t self_in) {
    mp_adc_stream_obj_t *self = adc_stream_from_obj(self_in);
    return MP_OBJ_NEW_SMALL_INT(adc_stream_hw_available(&self->hw));
}
static MP_DEFINE_CONST_FUN_OBJ_1(adc_stream_available_obj,
    adc_stream_available);

static mp_obj_t adc_stream_overruns(mp_obj_t self_in) {
    mp_adc_stream_obj_t *self = adc_stream_from_obj(self_in);
    return mp_obj_new_int_from_uint(adc_stream_hw_overruns(&self->hw));
}
static MP_DEFINE_CONST_FUN_OBJ_1(adc_stream_overruns_obj,
    adc_stream_overruns);

static mp_obj_t adc_stream_error(mp_obj_t self_in) {
    mp_adc_stream_obj_t *self = adc_stream_from_obj(self_in);
    return mp_obj_new_int_from_uint(adc_stream_hw_error(&self->hw));
}
static MP_DEFINE_CONST_FUN_OBJ_1(adc_stream_error_obj, adc_stream_error);

static mp_obj_t adc_stream_running(mp_obj_t self_in) {
    mp_adc_stream_obj_t *self = adc_stream_from_obj(self_in);
    return mp_obj_new_bool(adc_stream_hw_running(&self->hw));
}
static MP_DEFINE_CONST_FUN_OBJ_1(adc_stream_running_obj,
    adc_stream_running);

static bool adc_stream_valid_acquisition_cycles(mp_int_t cycles) {
    return cycles == 3 || cycles == 15 || cycles == 28 || cycles == 56
        || cycles == 84 || cycles == 112 || cycles == 144 || cycles == 480;
}

static mp_obj_t adc_stream_set_acquisition_cycles(mp_obj_t self_in,
    mp_obj_t cycles_in) {
    mp_adc_stream_obj_t *self = adc_stream_from_obj(self_in);
    if (self->hw.dma_initialised) {
        mp_raise_msg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("stop ADC stream first"));
    }
    mp_int_t cycles = mp_obj_get_int(cycles_in);
    if (!adc_stream_valid_acquisition_cycles(cycles)) {
        mp_raise_ValueError(MP_ERROR_TEXT(
            "cycles must be 3, 15, 28, 56, 84, 112, 144, or 480"));
    }
    self->acquisition_cycles = (uint16_t)cycles;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(adc_stream_set_acquisition_cycles_obj,
    adc_stream_set_acquisition_cycles);

static mp_obj_t adc_stream_get_acquisition_cycles(mp_obj_t self_in) {
    mp_adc_stream_obj_t *self = adc_stream_from_obj(self_in);
    return MP_OBJ_NEW_SMALL_INT(self->acquisition_cycles);
}
static MP_DEFINE_CONST_FUN_OBJ_1(adc_stream_get_acquisition_cycles_obj,
    adc_stream_get_acquisition_cycles);

static const mp_rom_map_elem_t adc_stream_locals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___del__), MP_ROM_PTR(&adc_stream_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_start), MP_ROM_PTR(&adc_stream_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop), MP_ROM_PTR(&adc_stream_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&adc_stream_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_poll), MP_ROM_PTR(&adc_stream_poll_obj) },
    { MP_ROM_QSTR(MP_QSTR_available), MP_ROM_PTR(&adc_stream_available_obj) },
    { MP_ROM_QSTR(MP_QSTR_overruns), MP_ROM_PTR(&adc_stream_overruns_obj) },
    { MP_ROM_QSTR(MP_QSTR_error), MP_ROM_PTR(&adc_stream_error_obj) },
    { MP_ROM_QSTR(MP_QSTR_running), MP_ROM_PTR(&adc_stream_running_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_acquisition_cycles),
        MP_ROM_PTR(&adc_stream_set_acquisition_cycles_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_acquisition_cycles),
        MP_ROM_PTR(&adc_stream_get_acquisition_cycles_obj) },
};
static MP_DEFINE_CONST_DICT(adc_stream_locals_dict, adc_stream_locals_table);

static MP_DEFINE_CONST_OBJ_TYPE(
    mp_adc_stream_type,
    MP_QSTR_ADCStream,
    MP_TYPE_FLAG_NONE,
    make_new, adc_stream_make_new,
    locals_dict, &adc_stream_locals_dict
);

static const mp_rom_map_elem_t adc_stream_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_adc_stream) },
    { MP_ROM_QSTR(MP_QSTR_ADCStream), MP_ROM_PTR(&mp_adc_stream_type) },
};
static MP_DEFINE_CONST_DICT(adc_stream_module_globals,
    adc_stream_module_globals_table);

const mp_obj_module_t adc_stream_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&adc_stream_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_adc_stream, adc_stream_user_cmodule);
