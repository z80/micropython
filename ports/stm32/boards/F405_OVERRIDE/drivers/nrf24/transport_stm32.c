#include "transport_stm32.h"

#include <string.h>

#include "extint.h"
#include "py/runtime.h"

#define TRANSPORT_STM32_GPIO_EXTI_LINES (16u)

static transport_stm32_t *volatile transport_stm32_exti_owners[
    TRANSPORT_STM32_GPIO_EXTI_LINES];

static uint32_t transport_stm32_timeout_cycles(void) {
    uint32_t cycles_per_us = SystemCoreClock / 1000000u;
    if (cycles_per_us == 0) {
        cycles_per_us = 1;
    }
    return cycles_per_us * TRANSPORT_STM32_SPI_TIMEOUT_US;
}

static bool transport_stm32_wait_set(volatile uint32_t *reg, uint32_t mask) {
    uint32_t start = (uint32_t)mp_hal_ticks_cpu();
    uint32_t timeout = transport_stm32_timeout_cycles();
    do {
        if ((*reg & mask) != 0) {
            return true;
        }
    } while ((uint32_t)((uint32_t)mp_hal_ticks_cpu() - start) < timeout);
    return false;
}

static bool transport_stm32_wait_clear(volatile uint32_t *reg, uint32_t mask) {
    uint32_t start = (uint32_t)mp_hal_ticks_cpu();
    uint32_t timeout = transport_stm32_timeout_cycles();
    do {
        if ((*reg & mask) == 0) {
            return true;
        }
    } while ((uint32_t)((uint32_t)mp_hal_ticks_cpu() - start) < timeout);
    return false;
}

static void transport_stm32_clear_overrun(SPI_TypeDef *instance) {
    if ((instance->SR & SPI_SR_OVR) != 0) {
        volatile uint32_t ignored = instance->DR;
        ignored = instance->SR;
        (void)ignored;
    }
}

static void transport_stm32_spi_fail(transport_stm32_t *adapter,
        transport_stm32_error_t error) {
    adapter->error = error;
    mp_hal_pin_write(adapter->cs, 1);
    transport_stm32_clear_overrun(adapter->spi->spi->Instance);
    adapter->spi_busy = false;
}

static mp_obj_t transport_stm32_exti_callback(mp_obj_t line_obj) {
    unsigned int line = (unsigned int)MP_OBJ_SMALL_INT_VALUE(line_obj);
    if (line < TRANSPORT_STM32_GPIO_EXTI_LINES) {
        transport_stm32_t *adapter = transport_stm32_exti_owners[line];
        if (adapter != NULL && adapter->irq_registered &&
                adapter->irq_handler != NULL) {
            adapter->irq_active = true;
            adapter->irq_count++;
            adapter->irq_handler(adapter->irq_context);
            adapter->irq_active = false;
        }
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(transport_stm32_exti_callback_obj,
    transport_stm32_exti_callback);

bool transport_stm32_init(transport_stm32_t *adapter, mp_obj_t spi_obj,
        mp_obj_t cs_obj, mp_obj_t ce_obj, mp_obj_t irq_obj) {
    const spi_t *spi;
    if (adapter == NULL) {
        return false;
    }

    memset(adapter, 0, sizeof(*adapter));
    adapter->exti_line = TRANSPORT_STM32_EXTI_NONE;
    adapter->spi_obj = spi_obj;
    adapter->cs_obj = cs_obj;
    adapter->ce_obj = ce_obj;
    adapter->irq_obj = irq_obj;

    spi = spi_from_mp_obj(spi_obj);
    adapter->spi = spi;
    adapter->cs = mp_hal_get_pin_obj(cs_obj);
    adapter->ce = mp_hal_get_pin_obj(ce_obj);
    adapter->irq = mp_hal_get_pin_obj(irq_obj);

    if (adapter->cs == adapter->ce || adapter->cs == adapter->irq ||
            adapter->ce == adapter->irq) {
        adapter->error = TRANSPORT_STM32_ERROR_ARGUMENT;
        return false;
    }
    if (spi == NULL || spi->spi == NULL || spi->spi->Instance == NULL ||
            spi->spi->Init.Mode != SPI_MODE_MASTER ||
            spi->spi->Init.Direction != SPI_DIRECTION_2LINES ||
            spi->spi->Init.DataSize != SPI_DATASIZE_8BIT ||
            spi->spi->Init.CLKPolarity != SPI_POLARITY_LOW ||
            spi->spi->Init.CLKPhase != SPI_PHASE_1EDGE ||
            spi->spi->Init.FirstBit != SPI_FIRSTBIT_MSB) {
        adapter->error = TRANSPORT_STM32_ERROR_SPI_CONFIGURATION;
        return false;
    }
    if (spi->spi->State == HAL_SPI_STATE_RESET ||
            (spi->spi->Instance->CR1 & SPI_CR1_SPE) == 0) {
        adapter->error = TRANSPORT_STM32_ERROR_SPI_NOT_READY;
        return false;
    }

    mp_hal_ticks_cpu_enable();
    mp_hal_pin_output(adapter->cs);
    mp_hal_pin_output(adapter->ce);
    mp_hal_pin_input(adapter->irq);
    mp_hal_pin_write(adapter->cs, 1);
    mp_hal_pin_write(adapter->ce, 0);
    adapter->initialized = true;
    return true;
}

void transport_stm32_deinit(transport_stm32_t *adapter) {
    if (adapter == NULL) {
        return;
    }
    transport_stm32_unregister_irq(adapter);
    if (adapter->initialized) {
        mp_hal_pin_write(adapter->ce, 0);
        mp_hal_pin_write(adapter->cs, 1);
        mp_hal_pin_input(adapter->irq);
    }
    adapter->irq_handler = NULL;
    adapter->irq_context = NULL;
    adapter->initialized = false;
    adapter->spi_busy = false;
}

bool transport_stm32_register_irq(transport_stm32_t *adapter,
        transport_stm32_irq_handler_t handler, void *context) {
    uint line;
    nlr_buf_t nlr;
    if (adapter == NULL || !adapter->initialized || handler == NULL ||
            adapter->irq_registered) {
        if (adapter != NULL) {
            adapter->error = TRANSPORT_STM32_ERROR_ARGUMENT;
        }
        return false;
    }

    /* extint_register checks MicroPython's ownership table and raises if the
       EXTI line is already used by another Pin or ExtInt object. */
    if (nlr_push(&nlr) != 0) {
        adapter->error = TRANSPORT_STM32_ERROR_EXTI_IN_USE;
        return false;
    }
    line = extint_register(adapter->irq_obj, GPIO_MODE_IT_FALLING,
        GPIO_NOPULL, MP_OBJ_FROM_PTR(&transport_stm32_exti_callback_obj), false);
    nlr_pop();
    if (line >= TRANSPORT_STM32_GPIO_EXTI_LINES) {
        extint_register(adapter->irq_obj, GPIO_MODE_IT_FALLING, GPIO_NOPULL,
            mp_const_none, true);
        adapter->error = TRANSPORT_STM32_ERROR_EXTI_IN_USE;
        return false;
    }

    adapter->irq_handler = handler;
    adapter->irq_context = context;
    adapter->exti_line = (uint8_t)line;
    adapter->irq_registered = true;
    /* extint_register already rejected any live owner. Overwrite a possible
       stale C pointer left by a MicroPython soft reset. */
    transport_stm32_exti_owners[line] = adapter;

    /* An active-low IRQ present before EXTI was armed does not produce a new
       falling edge. Service it once after publishing the owner. */
    if (mp_hal_pin_read(adapter->irq) == 0) {
        adapter->irq_active = true;
        adapter->irq_count++;
        adapter->irq_handler(adapter->irq_context);
        adapter->irq_active = false;
    }
    return true;
}

void transport_stm32_unregister_irq(transport_stm32_t *adapter) {
    uint8_t line;
    if (adapter == NULL || !adapter->irq_registered) {
        return;
    }
    line = adapter->exti_line;
    if (line < TRANSPORT_STM32_GPIO_EXTI_LINES &&
            transport_stm32_exti_owners[line] == adapter) {
        transport_stm32_exti_owners[line] = NULL;
    }
    adapter->irq_registered = false;
    adapter->irq_handler = NULL;
    adapter->irq_context = NULL;
    adapter->exti_line = TRANSPORT_STM32_EXTI_NONE;
    extint_register(adapter->irq_obj, GPIO_MODE_IT_FALLING, GPIO_NOPULL,
        mp_const_none, true);
    mp_hal_pin_input(adapter->irq);
}

void transport_stm32_enable_irq(transport_stm32_t *adapter) {
    if (adapter != NULL && adapter->irq_registered) {
        extint_enable(adapter->exti_line);
    }
}

void transport_stm32_disable_irq(transport_stm32_t *adapter) {
    if (adapter != NULL && adapter->irq_registered) {
        extint_disable(adapter->exti_line);
    }
}

void transport_stm32_cs_write(transport_stm32_t *adapter, bool high) {
    if (adapter != NULL && adapter->initialized) {
        mp_hal_pin_write(adapter->cs, high);
    }
}

void transport_stm32_ce_write(transport_stm32_t *adapter, bool high) {
    if (adapter != NULL && adapter->initialized) {
        mp_hal_pin_write(adapter->ce, high);
    }
}

bool transport_stm32_irq_is_low(const transport_stm32_t *adapter) {
    return adapter != NULL && adapter->initialized &&
        mp_hal_pin_read(adapter->irq) == 0;
}

bool transport_stm32_spi_transfer(transport_stm32_t *adapter, size_t length,
        const uint8_t *tx, uint8_t *rx) {
    SPI_HandleTypeDef *handle;
    SPI_TypeDef *instance;
    size_t i;
    if (adapter == NULL || !adapter->initialized) {
        if (adapter != NULL) {
            adapter->error = TRANSPORT_STM32_ERROR_ARGUMENT;
        }
        return false;
    }
    if (length == 0) {
        return true;
    }
    if (adapter->spi_busy) {
        adapter->error = TRANSPORT_STM32_ERROR_SPI_BUSY;
        return false;
    }

    handle = adapter->spi->spi;
    instance = handle->Instance;
    if (handle->State != HAL_SPI_STATE_READY) {
        adapter->error = TRANSPORT_STM32_ERROR_SPI_BUSY;
        return false;
    }
    if ((instance->CR1 & SPI_CR1_SPE) == 0) {
        adapter->error = TRANSPORT_STM32_ERROR_SPI_NOT_READY;
        return false;
    }

    adapter->spi_busy = true;
    transport_stm32_clear_overrun(instance);
    for (i = 0; i < length; ++i) {
        uint8_t received;
        if (!transport_stm32_wait_set(&instance->SR, SPI_SR_TXE)) {
            transport_stm32_spi_fail(adapter,
                TRANSPORT_STM32_ERROR_SPI_TX_TIMEOUT);
            return false;
        }
        *(__IO uint8_t *)&instance->DR = tx == NULL ? 0xffu : tx[i];
        if (!transport_stm32_wait_set(&instance->SR, SPI_SR_RXNE)) {
            transport_stm32_spi_fail(adapter,
                TRANSPORT_STM32_ERROR_SPI_RX_TIMEOUT);
            return false;
        }
        received = *(__IO uint8_t *)&instance->DR;
        if (rx != NULL) {
            rx[i] = received;
        }
    }
    if (!transport_stm32_wait_set(&instance->SR, SPI_SR_TXE) ||
            !transport_stm32_wait_clear(&instance->SR, SPI_SR_BSY)) {
        transport_stm32_spi_fail(adapter,
            TRANSPORT_STM32_ERROR_SPI_BSY_TIMEOUT);
        return false;
    }

    adapter->spi_busy = false;
    return true;
}

transport_stm32_error_t transport_stm32_get_error(
        const transport_stm32_t *adapter) {
    return adapter == NULL ? TRANSPORT_STM32_ERROR_ARGUMENT : adapter->error;
}

void transport_stm32_clear_error(transport_stm32_t *adapter) {
    if (adapter != NULL) {
        adapter->error = TRANSPORT_STM32_ERROR_NONE;
    }
}
