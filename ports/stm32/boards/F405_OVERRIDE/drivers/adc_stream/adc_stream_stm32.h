#ifndef MICROPY_INCLUDED_F405_OVERRIDE_ADC_STREAM_STM32_H
#define MICROPY_INCLUDED_F405_OVERRIDE_ADC_STREAM_STM32_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "py/mphal.h"
#include "timer.h"

typedef struct _adc_stream_hw_t {
    ADC_HandleTypeDef adc;
    DMA_HandleTypeDef dma;
    uint16_t *buffer;
    size_t sample_count;
    volatile uint8_t ready_mask;
    volatile uint32_t overruns;
    volatile uint32_t error;
    bool dma_initialised;
    volatile bool running;
} adc_stream_hw_t;

// Start ADC1 sampling from the supplied timer's update TRGO into buffer.
// The timer remains owned by its Python pyb.Timer object.
bool adc_stream_hw_start(adc_stream_hw_t *self, uint32_t channel,
    TIM_HandleTypeDef *timer, uint32_t acquisition_cycles,
    uint16_t *buffer, size_t sample_count);

void adc_stream_hw_stop(adc_stream_hw_t *self);
int adc_stream_hw_take_ready(adc_stream_hw_t *self);
uint8_t adc_stream_hw_available(adc_stream_hw_t *self);
uint32_t adc_stream_hw_overruns(adc_stream_hw_t *self);
uint32_t adc_stream_hw_error(adc_stream_hw_t *self);
bool adc_stream_hw_running(adc_stream_hw_t *self);

// Called by the board soft-reset hook before timers and the GC heap are reset.
void adc_stream_deinit_all(void);

#endif // MICROPY_INCLUDED_F405_OVERRIDE_ADC_STREAM_STM32_H
