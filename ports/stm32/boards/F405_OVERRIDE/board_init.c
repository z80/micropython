#include "mpconfigboard.h"
#include "py/mphal.h"
#include "spi.h"

const spi_t *spi2 = &spi_obj[1];

void F405_SERGEY_V2_board_early_init(void) {
    // Ensure CS is high before the SPI peripheral is configured
    mp_hal_pin_output(MICROPY_HW_SPIFLASH_CS);
    mp_hal_pin_write(MICROPY_HW_SPIFLASH_CS, 1);

    mp_hal_pin_output(MICROPY_HW_LED1);
    mp_hal_pin_output(MICROPY_HW_LED2);

    return;

    /*
    spi_init0();

    for (;;) {
        static const uint8_t cmd = 0x9F;
        uint8_t response[3] = {0};

        mp_hal_pin_write(MICROPY_HW_LED1, 1);  // LED0 on: SPI active
        mp_hal_pin_write(MICROPY_HW_LED2, 0);  // LED1 off

        spi_transfer(spi2, 1, &cmd, NULL, 100);
        spi_transfer(spi2, 3, NULL, response, 100);

        mp_hal_pin_write(MICROPY_HW_LED1, 0);  // LED0 off
        mp_hal_pin_write(MICROPY_HW_LED2, 1);  // LED1 on: idle

        mp_hal_delay_ms(10);  // 10ms pause
    }*/
}

