#include "storage.h"
#include "spi.h"

#if !MICROPY_HW_ENABLE_INTERNAL_FLASH_STORAGE

static mp_spiflash_cache_t spi_bdev_cache;

const mp_spiflash_config_t spiflash_config = {
    .bus_kind = MP_SPIFLASH_BUS_SPI,
    .bus.u_spi.cs = MICROPY_HW_SPIFLASH_CS,
    .bus.u_spi.data = {
        .spi = &spi_obj[1], // SPI2 is index 1
    },
    .cache = &spi_bdev_cache,
};

spi_bdev_t spi_bdev;

#endif

