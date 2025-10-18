#define MICROPY_HW_BOARD_NAME       "F405_SERGEY_V2"
#define MICROPY_HW_MCU_NAME         "STM32F405RG"

// PLL configuration: 8 MHz HSE → 168 MHz SYSCLK, 48 MHz USB
#define MICROPY_HW_CLK_PLLM (8)
#define MICROPY_HW_CLK_PLLN (336)
#define MICROPY_HW_CLK_PLLP (RCC_PLLP_DIV2)
#define MICROPY_HW_CLK_PLLQ (7)

// LEDs
#define MICROPY_HW_LED1             (pin_A15)
#define MICROPY_HW_LED2             (pin_C10)
#define MICROPY_HW_LED_INVERTED     (0)
#define MICROPY_HW_LED_ON(pin)      (mp_hal_pin_low(pin))
#define MICROPY_HW_LED_OFF(pin)     (mp_hal_pin_high(pin))


// USB FS (D+ / D- only)
#define MICROPY_HW_USB_FS           (1)
#define MICROPY_HW_USB_VBUS_DETECT_PIN (0)
#define MICROPY_HW_USB_OTG_ID_PIN      (0)

// External SPI Flash (W25Q128JV, 128 Mbit = 16 MB)
#define MICROPY_HW_SPIFLASH_SIZE_BITS (128 * 1024 * 1024)
#define MICROPY_HW_SPIFLASH_CS      (pin_B12)
#define MICROPY_HW_SPIFLASH_SCK     (pin_B13)
#define MICROPY_HW_SPIFLASH_MOSI    (pin_C3)
#define MICROPY_HW_SPIFLASH_MISO    (pin_C2)
#define MICROPY_HW_SPIFLASH_BUS     (2) // SPI2

// Block device config
#define MICROPY_HW_BDEV_SPIFLASH_CONFIG (&spiflash_config)
extern const struct _mp_spiflash_config_t spiflash_config;

