#include "storage.h"
#include "spi.h"

#if !MICROPY_HW_ENABLE_INTERNAL_FLASH

#if MICROPY_HW_SPIFLASH_ENABLE_CACHE
STATIC mp_spiflash_cache_t spi_bdev_cache;
#endif

const mp_spiflash_config_t spiflash_config = {
    .bus_kind = MP_SPIFLASH_BUS_SPI,
    .bus.u_spi = {
        .cs = MICROPY_HW_SPIFLASH_CS,
        .data = (void *)&spi_obj[MICROPY_HW_SPIFLASH_BUS - 1], // cast fixes const mismatch
        .proto = &spi_proto,
    },
    #if MICROPY_HW_SPIFLASH_ENABLE_CACHE
    .cache = &spi_bdev_cache,
    #endif
};

spi_bdev_t spi_bdev;

#endif

