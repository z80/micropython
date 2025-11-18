#include "py/obj.h"
#include "py/runtime.h"
#include "py/objtype.h"
#include "py/builtin.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "extmod/virtpin.h"

#include "i2c.h"

#include "bmi08.h"
#include "bmi08x.h"
#include "bmi08_defs.h"

#ifndef STATIC
#define STATIC static
#endif

#define I2C_TIMEOUT     168000000
#define BMI08_READ_WRITE_LEN  UINT8_C(46)

typedef struct _bmi08_i2c_ctx_t {
    mp_obj_t i2c_obj;
    uint8_t addr;
} bmi08_i2c_ctx_t;


typedef struct _mp_bmi08_obj_t {
    mp_obj_base_t base;
    struct bmi08_dev dev;

    bmi08_i2c_ctx_t acc_ctx;
    bmi08_i2c_ctx_t gyro_ctx;
} mp_bmi08_obj_t;








int8_t bmi08_hal_read( uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
    bmi08_i2c_ctx_t *ctx = (bmi08_i2c_ctx_t *)intf_ptr;
    pyb_i2c_obj_t *i2c = MP_OBJ_TO_PTR(ctx->i2c_obj);
    I2C_HandleTypeDef *hi2c = i2c->i2c;

    HAL_StatusTypeDef res = HAL_I2C_Mem_Read(
        hi2c,
        ctx->addr << 1,
        reg_addr,
        I2C_MEMADD_SIZE_8BIT,
        data,
        len,
        I2C_TIMEOUT
    );

    return (res == HAL_OK) ? BMI08_INTF_RET_SUCCESS : (BMI08_INTF_RET_SUCCESS + 1);
}



int8_t bmi08_hal_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
    bmi08_i2c_ctx_t *ctx = (bmi08_i2c_ctx_t *)intf_ptr;
    pyb_i2c_obj_t *i2c = MP_OBJ_TO_PTR(ctx->i2c_obj);
    I2C_HandleTypeDef *hi2c = i2c->i2c;

    HAL_StatusTypeDef res = HAL_I2C_Mem_Write(
        hi2c,
        ctx->addr << 1,
        reg_addr,
        I2C_MEMADD_SIZE_8BIT,
        (uint8_t *)data,
        len,
        I2C_TIMEOUT
    );

    return (res == HAL_OK) ? BMI08_INTF_RET_SUCCESS : (BMI08_INTF_RET_SUCCESS + 1);
}


STATIC void bmi08_delay_us(uint32_t period, void *intf_ptr) {
    mp_hal_delay_us(period);
}



STATIC void bmi08_check_rslt(int8_t rslt, const char *msg) {
    if (rslt != BMI08_OK) {
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("BMI08 init failed: %s (code %d)"), msg, rslt);
    }
}











// Constructor.
STATIC mp_obj_t bmi08_make_new(const mp_obj_type_t *type,
                                size_t n_args, size_t n_kw,
                                const mp_obj_t *args) {
    enum { ARG_i2c, ARG_use_primary_acc, ARG_use_primary_gyro };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_i2c, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_use_primary_acc, MP_ARG_BOOL | MP_ARG_KW_ONLY, {.u_bool = true} },
        { MP_QSTR_use_primary_gyro, MP_ARG_BOOL | MP_ARG_KW_ONLY, {.u_bool = true} },
    };

    mp_arg_val_t parsed_args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, args, NULL, MP_ARRAY_SIZE(allowed_args), allowed_args, parsed_args);

    mp_obj_t i2c_obj = parsed_args[ARG_i2c].u_obj;

    bool use_primary_acc = parsed_args[ARG_use_primary_acc].u_bool;
    bool use_primary_gyro = parsed_args[ARG_use_primary_gyro].u_bool;

    mp_bmi08_obj_t *self = m_new_obj(mp_bmi08_obj_t);
    self->base.type = type;

    self->acc_ctx.i2c_obj = i2c_obj;
    self->acc_ctx.addr = use_primary_acc ? BMI08_ACCEL_I2C_ADDR_PRIMARY : BMI08_ACCEL_I2C_ADDR_SECONDARY;

    self->gyro_ctx.i2c_obj = i2c_obj;
    self->gyro_ctx.addr = use_primary_gyro ? BMI08_GYRO_I2C_ADDR_PRIMARY : BMI08_GYRO_I2C_ADDR_SECONDARY;

    self->dev.intf_ptr_accel = &self->acc_ctx;
    self->dev.intf_ptr_gyro  = &self->gyro_ctx;

    self->dev.intf = BMI08_I2C_INTF;
    self->dev.read = bmi08_hal_read;
    self->dev.write = bmi08_hal_write;
    self->dev.delay_us = bmi08_delay_us;
    self->dev.read_write_len = BMI08_READ_WRITE_LEN;
    self->dev.variant = BMI085_VARIANT;

    return MP_OBJ_FROM_PTR(self);
}

// Initializing the IMU.
STATIC mp_obj_t bmi08_init(mp_obj_t self_in) {
    mp_bmi08_obj_t *self = MP_OBJ_TO_PTR(self_in);
    struct bmi08_dev *bmi08 = &self->dev;

    bmi08_check_rslt(bmi08a_soft_reset(bmi08), "accelerometer soft reset");
    bmi08_check_rslt(bmi08g_soft_reset(bmi08), "gyroscope soft reset");

    bmi08_check_rslt(bmi08xa_init(bmi08), "variant init");
    bmi08_check_rslt(bmi08a_init(bmi08), "accelerometer init");
    bmi08_check_rslt(bmi08g_init(bmi08), "gyroscope init");

    bmi08_check_rslt(bmi08a_load_config_file(bmi08), "load config file");

    // Accelerometer config
    bmi08->accel_cfg.odr   = BMI08_ACCEL_ODR_100_HZ;
    bmi08->accel_cfg.range = BMI085_ACCEL_RANGE_8G;
    bmi08->accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
    bmi08->accel_cfg.bw    = BMI08_ACCEL_BW_NORMAL;

    bmi08_check_rslt(bmi08a_set_power_mode(bmi08), "set accel power mode");
    bmi08_check_rslt(bmi08xa_set_meas_conf(bmi08), "set accel meas conf");

    // Gyroscope config
    bmi08->gyro_cfg.odr   = BMI08_GYRO_BW_47_ODR_400_HZ;
    bmi08->gyro_cfg.range = BMI08_GYRO_RANGE_250_DPS;
    bmi08->gyro_cfg.bw    = BMI08_GYRO_BW_47_ODR_400_HZ;
    bmi08->gyro_cfg.power = BMI08_GYRO_PM_NORMAL;

    bmi08_check_rslt(bmi08g_set_power_mode(bmi08), "set gyro power mode");
    bmi08_check_rslt(bmi08g_set_meas_conf(bmi08), "set gyro meas conf");

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(bmi08_init_obj, bmi08_init);


STATIC mp_obj_t bmi08_read_acc(mp_obj_t self_in) {
    mp_bmi08_obj_t *self = MP_OBJ_TO_PTR(self_in);
    struct bmi08_sensor_data acc;
    bmi08a_get_data(&acc, &self->dev);

    mp_obj_t tuple[3] = {
        mp_obj_new_int(acc.x),
        mp_obj_new_int(acc.y),
        mp_obj_new_int(acc.z),
    };
    return mp_obj_new_tuple(3, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(bmi08_read_acc_obj, bmi08_read_acc);


STATIC mp_obj_t bmi08_read_gyro(mp_obj_t self_in) {
    mp_bmi08_obj_t *self = MP_OBJ_TO_PTR(self_in);
    struct bmi08_sensor_data gyro;

    int8_t rslt = bmi08g_get_data(&gyro, &self->dev);
    if (rslt != BMI08_OK) {
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("BMI08 gyro read failed (code %d)"), rslt);
    }

    mp_obj_t tuple[3] = {
        mp_obj_new_int(gyro.x),
        mp_obj_new_int(gyro.y),
        mp_obj_new_int(gyro.z)
    };
    return mp_obj_new_tuple(3, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(bmi08_read_gyro_obj, bmi08_read_gyro);



STATIC mp_obj_t bmi08_init_gyro_fifo(mp_obj_t self_in) {
    mp_bmi08_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int8_t rslt;

    // 1. Initialize gyro interface
    rslt = bmi08g_init(&self->dev);
    if (rslt != BMI08_OK) {
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("Gyro init failed: %d"), rslt);
    }

    // 2. Set gyro power mode
    self->dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
    rslt = bmi08g_set_power_mode(&self->dev);
    if (rslt != BMI08_OK) {
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("Gyro power mode failed: %d"), rslt);
    }

    // 3. Set gyro ODR and range (combined ODR+BW constant)
    self->dev.gyro_cfg.odr = BMI08_GYRO_BW_47_ODR_400_HZ;
    self->dev.gyro_cfg.range = BMI08_GYRO_RANGE_2000_DPS;

    rslt = bmi08g_set_meas_conf(&self->dev);
    if (rslt != BMI08_OK) {
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("Gyro meas config failed: %d"), rslt);
    }

    // 4. Configure gyro FIFO (headerless, XYZ data only)
    struct bmi08_gyr_fifo_config fifo_cfg = {0};
    fifo_cfg.mode = BMI08_GYRO_FIFO_MODE;          // FIFO mode
    fifo_cfg.data_select = BMI08_GYRO_FIFO_XYZ_AXIS_ENABLED;    // Enable XYZ gyro data
    fifo_cfg.tag = BMI08_DISABLE;                  // Headerless mode
    fifo_cfg.frame_count = 0;                      // Not used during config
    fifo_cfg.wm_level = 0;                         // Optional: set if using watermark interrupt


    rslt = bmi08g_set_fifo_config(&fifo_cfg, &self->dev);
    if (rslt != BMI08_OK) {
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("Gyro FIFO config failed: %d"), rslt);
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(bmi08_init_gyro_fifo_obj, bmi08_init_gyro_fifo);



STATIC mp_obj_t bmi08_read_gyro_sum(mp_obj_t self_in) {
    mp_bmi08_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int8_t rslt;

    // 1. Prepare FIFO config (must match init)
    struct bmi08_gyr_fifo_config fifo_cfg = {
        .mode = BMI08_GYRO_FIFO_MODE,
        .data_select = BMI08_GYRO_FIFO_XYZ_AXIS_ENABLED,
        .tag = BMI08_DISABLE,
        .frame_count = 0,
        .wm_level = 0
    };

    // 2. Prepare FIFO frame buffer
    uint8_t fifo_data[256] = {0};  // adjust size as needed
    struct bmi08_fifo_frame fifo = {
        .data = fifo_data,
        .length = sizeof(fifo_data)
    };

    // 3. Get FIFO length
    rslt = bmi08g_get_fifo_length(&fifo_cfg, &fifo);
    if (rslt != BMI08_OK || fifo.length == 0) {
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("FIFO length error: %d"), rslt);
    }

    // 4. Read FIFO data
    rslt = bmi08g_read_fifo_data(&fifo, &self->dev);
    if (rslt != BMI08_OK) {
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("FIFO read error: %d"), rslt);
    }

    // 5. Extract gyro frames
    struct bmi08_sensor_data gyro_frames[32] = {0};  // adjust size as needed
    uint16_t frame_count = 0;

    bmi08g_extract_gyro(gyro_frames, &frame_count, &fifo_cfg, &fifo);

    // 6. Accumulate angular deltas
    int32_t x_sum = 0, y_sum = 0, z_sum = 0;
    for (uint16_t i = 0; i < frame_count; ++i) {
        x_sum += gyro_frames[i].x;
        y_sum += gyro_frames[i].y;
        z_sum += gyro_frames[i].z;
    }

    // Return tuple: (x_sum, y_sum, z_sum, frame_count)
    // scale for 2000dps at 200 Hz
    // scale = x / 32768 * 2000 / 200 = 0.00030517578125
    const float SCALE = 0.00030517578125;
    mp_obj_t tuple[4] = {
        mp_obj_new_int( (int)( (float)(x_sum) * SCALE ) ),
        mp_obj_new_int( (int)( (float)(y_sum) * SCALE ) ),
        mp_obj_new_int( (int)( (float)(z_sum) * SCALE ) ), 
        mp_obj_new_int(frame_count)
    };
    return mp_obj_new_tuple(4, tuple);

}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(bmi08_read_gyro_sum_obj, bmi08_read_gyro_sum);



// Methods table.
STATIC const mp_rom_map_elem_t bmi08_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&bmi08_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_acc), MP_ROM_PTR(&bmi08_read_acc_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_gyro), MP_ROM_PTR(&bmi08_read_gyro_obj) },
    { MP_ROM_QSTR(MP_QSTR_init_gyro_fifo), MP_ROM_PTR(&bmi08_init_gyro_fifo_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_gyro_sum), MP_ROM_PTR(&bmi08_read_gyro_sum_obj) },
};
STATIC MP_DEFINE_CONST_DICT(bmi08_locals_dict, bmi08_locals_dict_table);


STATIC const mp_obj_type_t bmi08_type = {
    .base = { &mp_type_type },
    .flags = 0,
    .name = MP_QSTR_BMI08,
    .slot_index_make_new = 0,
    .slot_index_locals_dict = 1,
    .slots = {
        (void *)bmi08_make_new,
        (void *)&bmi08_locals_dict,
    },
};


// Object itself.
STATIC const mp_rom_map_elem_t bmi08_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_BMI08), MP_ROM_PTR(&bmi08_type) },
};
STATIC MP_DEFINE_CONST_DICT(bmi08_module_globals, bmi08_module_globals_table);

const mp_obj_module_t bmi08_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&bmi08_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_bmi08, bmi08_user_cmodule);


