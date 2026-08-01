#ifndef TRANSPORT_STM32_H
#define TRANSPORT_STM32_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "py/mphal.h"
#include "py/obj.h"
#include "spi.h"

#ifndef TRANSPORT_STM32_SPI_TIMEOUT_US
#define TRANSPORT_STM32_SPI_TIMEOUT_US (1000u)
#endif

#define TRANSPORT_STM32_EXTI_NONE (0xffu)

typedef enum {
    TRANSPORT_STM32_ERROR_NONE = 0,
    TRANSPORT_STM32_ERROR_ARGUMENT,
    TRANSPORT_STM32_ERROR_SPI_CONFIGURATION,
    TRANSPORT_STM32_ERROR_SPI_NOT_READY,
    TRANSPORT_STM32_ERROR_SPI_BUSY,
    TRANSPORT_STM32_ERROR_SPI_TX_TIMEOUT,
    TRANSPORT_STM32_ERROR_SPI_RX_TIMEOUT,
    TRANSPORT_STM32_ERROR_SPI_BSY_TIMEOUT,
    TRANSPORT_STM32_ERROR_EXTI_IN_USE,
} transport_stm32_error_t;

typedef void (*transport_stm32_irq_handler_t)(void *context);

typedef struct {
    /* Object references keep the Python-owned hardware objects alive while
       this structure is embedded in a GC-managed binding object. */
    mp_obj_t spi_obj;
    mp_obj_t cs_obj;
    mp_obj_t ce_obj;
    mp_obj_t irq_obj;

    const spi_t *spi;
    mp_hal_pin_obj_t cs;
    mp_hal_pin_obj_t ce;
    mp_hal_pin_obj_t irq;

    transport_stm32_irq_handler_t irq_handler;
    void *irq_context;
    volatile transport_stm32_error_t error;
    volatile uint32_t irq_count;
    volatile bool spi_busy;
    volatile bool irq_active;
    uint8_t exti_line;
    bool initialized;
    bool irq_registered;
} transport_stm32_t;

/* Extracts an already configured hardware SPI and configures only CS, CE and
   IRQ GPIO directions. It never calls spi_init() or spi_deinit(). */
bool transport_stm32_init(transport_stm32_t *adapter, mp_obj_t spi_obj,
    mp_obj_t cs_obj, mp_obj_t ce_obj, mp_obj_t irq_obj);
void transport_stm32_deinit(transport_stm32_t *adapter);

/* Registers a falling-edge hard EXTI callback. The handler normally runs in
   interrupt context, but may run synchronously once during registration if
   IRQ is already low. It must not allocate, raise, or call Python code. */
bool transport_stm32_register_irq(transport_stm32_t *adapter,
    transport_stm32_irq_handler_t handler, void *context);
void transport_stm32_unregister_irq(transport_stm32_t *adapter);
void transport_stm32_enable_irq(transport_stm32_t *adapter);
void transport_stm32_disable_irq(transport_stm32_t *adapter);

void transport_stm32_cs_write(transport_stm32_t *adapter, bool high);
void transport_stm32_ce_write(transport_stm32_t *adapter, bool high);
bool transport_stm32_irq_is_low(const transport_stm32_t *adapter);

/* Allocation-free, bounded, full-duplex polling transfer suitable for the
   EXTI handler. A NULL TX pointer sends 0xff; a NULL RX pointer discards data. */
bool transport_stm32_spi_transfer(transport_stm32_t *adapter, size_t length,
    const uint8_t *tx, uint8_t *rx);

transport_stm32_error_t transport_stm32_get_error(
    const transport_stm32_t *adapter);
void transport_stm32_clear_error(transport_stm32_t *adapter);

#endif
