#include "storage.h"
#include "spi.h"

#if !MICROPY_HW_ENABLE_INTERNAL_FLASH_STORAGE

#if MICROPY_HW_SPIFLASH_ENABLE_CACHE
static mp_spiflash_cache_t spi_bdev_cache;
#endif


static const spi_proto_cfg_t spi_bus = {
    .spi = &spi_obj[MICROPY_HW_SPIFLASH_BUS - 1],
    .baudrate = 25000000,
    .polarity = 0,
    .phase = 0,
    .bits = 8,
    .firstbit = SPI_FIRSTBIT_MSB,
};

const mp_spiflash_config_t spiflash_config = {
    .bus_kind = MP_SPIFLASH_BUS_SPI,
    .bus.u_spi = {
        .cs = MICROPY_HW_SPIFLASH_CS,
        .data = (void *)&spi_bus, 
        .proto = &spi_proto,
    },
    #if MICROPY_HW_SPIFLASH_ENABLE_CACHE
    .cache = &spi_bdev_cache,
    #endif
};

spi_bdev_t spi_bdev;

#endif

