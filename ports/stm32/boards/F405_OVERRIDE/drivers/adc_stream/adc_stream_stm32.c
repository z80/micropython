#include <string.h>

#include "py/mphal.h"
#include "dma.h"
#include "adc_stream_stm32.h"

static adc_stream_hw_t *volatile adc_stream_active;

static bool adc_stream_timer_trigger(TIM_HandleTypeDef *timer,
    uint32_t *trigger) {
    if (timer == NULL || timer->State == HAL_TIM_STATE_RESET) {
        return false;
    }
    if (timer->Instance == TIM2) {
        *trigger = ADC_EXTERNALTRIGCONV_T2_TRGO;
    } else if (timer->Instance == TIM3) {
        *trigger = ADC_EXTERNALTRIGCONV_T3_TRGO;
    } else if (timer->Instance == TIM8) {
        *trigger = ADC_EXTERNALTRIGCONV_T8_TRGO;
    } else {
        return false;
    }
    return true;
}

static bool adc_stream_sample_time(uint32_t cycles, uint32_t *sample_time) {
    switch (cycles) {
        case 3:
            *sample_time = ADC_SAMPLETIME_3CYCLES;
            break;
        case 15:
            *sample_time = ADC_SAMPLETIME_15CYCLES;
            break;
        case 28:
            *sample_time = ADC_SAMPLETIME_28CYCLES;
            break;
        case 56:
            *sample_time = ADC_SAMPLETIME_56CYCLES;
            break;
        case 84:
            *sample_time = ADC_SAMPLETIME_84CYCLES;
            break;
        case 112:
            *sample_time = ADC_SAMPLETIME_112CYCLES;
            break;
        case 144:
            *sample_time = ADC_SAMPLETIME_144CYCLES;
            break;
        case 480:
            *sample_time = ADC_SAMPLETIME_480CYCLES;
            break;
        default:
            return false;
    }
    return true;
}

static void adc_stream_release_active(adc_stream_hw_t *self) {
    mp_uint_t atomic_state = MICROPY_BEGIN_ATOMIC_SECTION();
    if (adc_stream_active == self) {
        adc_stream_active = NULL;
    }
    MICROPY_END_ATOMIC_SECTION(atomic_state);
}

bool adc_stream_hw_start(adc_stream_hw_t *self, uint32_t channel,
    TIM_HandleTypeDef *timer, uint32_t acquisition_cycles,
    uint16_t *buffer, size_t sample_count) {
    uint32_t trigger;
    uint32_t sample_time;
    if (self == NULL || buffer == NULL || sample_count < 2
        || sample_count > UINT16_MAX || (sample_count & 1) != 0
        || !adc_stream_timer_trigger(timer, &trigger)
        || !adc_stream_sample_time(acquisition_cycles, &sample_time)) {
        return false;
    }

    // A DMA error leaves the resources initialised so error() remains useful.
    // Tear that state down before allowing an explicit restart.
    if (self->dma_initialised) {
        adc_stream_hw_stop(self);
    }

    mp_uint_t atomic_state = MICROPY_BEGIN_ATOMIC_SECTION();
    if (adc_stream_active != NULL && adc_stream_active != self) {
        MICROPY_END_ATOMIC_SECTION(atomic_state);
        return false;
    }
    adc_stream_active = self;
    MICROPY_END_ATOMIC_SECTION(atomic_state);

    self->buffer = buffer;
    self->sample_count = sample_count;
    self->ready_mask = 0;
    self->overruns = 0;
    self->error = HAL_ADC_ERROR_NONE;

    TIM_MasterConfigTypeDef timer_config;
    memset(&timer_config, 0, sizeof(timer_config));
    timer_config.MasterOutputTrigger = TIM_TRGO_UPDATE;
    timer_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(timer, &timer_config) != HAL_OK) {
        adc_stream_release_active(self);
        return false;
    }

    __HAL_RCC_ADC1_CLK_ENABLE();
    memset(&self->adc, 0, sizeof(self->adc));
    self->adc.Instance = ADC1;
    self->adc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    self->adc.Init.Resolution = ADC_RESOLUTION_12B;
    self->adc.Init.ScanConvMode = DISABLE;
    self->adc.Init.ContinuousConvMode = DISABLE;
    self->adc.Init.DiscontinuousConvMode = DISABLE;
    self->adc.Init.NbrOfDiscConversion = 0;
    self->adc.Init.ExternalTrigConv = trigger;
    self->adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    self->adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    self->adc.Init.NbrOfConversion = 1;
    self->adc.Init.DMAContinuousRequests = ENABLE;
    self->adc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&self->adc) != HAL_OK) {
        HAL_ADC_DeInit(&self->adc);
        self->adc.Instance = NULL;
        adc_stream_release_active(self);
        return false;
    }

    ADC_ChannelConfTypeDef channel_config;
    memset(&channel_config, 0, sizeof(channel_config));
    channel_config.Channel = channel;
    channel_config.Rank = 1;
    channel_config.SamplingTime = sample_time;
    if (HAL_ADC_ConfigChannel(&self->adc, &channel_config) != HAL_OK) {
        HAL_ADC_DeInit(&self->adc);
        self->adc.Instance = NULL;
        adc_stream_release_active(self);
        return false;
    }

    dma_init(&self->dma, &dma_ADC_1_RX, DMA_PERIPH_TO_MEMORY, &self->adc);
    self->dma_initialised = true;
    self->adc.DMA_Handle = &self->dma;
    self->running = true;

    if (HAL_ADC_Start_DMA(&self->adc, (uint32_t *)buffer,
        (uint32_t)sample_count) != HAL_OK) {
        adc_stream_hw_stop(self);
        return false;
    }

    // The STM32F4 HAL enables ADC_IT_OVR, but this stream is serviced through
    // the DMA IRQ and this port has no ADC_IRQHandler. Keep the sticky OVR flag
    // available to error() without enabling an unhandled interrupt source.
    __HAL_ADC_DISABLE_IT(&self->adc, ADC_IT_OVR);

    return true;
}

void adc_stream_hw_stop(adc_stream_hw_t *self) {
    if (self == NULL) {
        return;
    }

    if (self->dma_initialised) {
        HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
        adc_stream_release_active(self);
        if (__HAL_ADC_GET_FLAG(&self->adc, ADC_FLAG_OVR)) {
            self->error |= HAL_ADC_ERROR_OVR;
        }
        HAL_ADC_Stop_DMA(&self->adc);
        dma_deinit(&dma_ADC_1_RX);
        self->dma_initialised = false;
    } else {
        adc_stream_release_active(self);
    }
    if (self->adc.Instance == ADC1) {
        HAL_ADC_DeInit(&self->adc);
        self->adc.Instance = NULL;
        self->adc.DMA_Handle = NULL;
    }
    self->running = false;
    self->ready_mask = 0;
}

int adc_stream_hw_take_ready(adc_stream_hw_t *self) {
    mp_uint_t atomic_state = MICROPY_BEGIN_ATOMIC_SECTION();
    uint8_t ready = self->ready_mask;
    int half = -1;
    if (ready & 0x01) {
        half = 0;
        self->ready_mask = ready & ~0x01;
    } else if (ready & 0x02) {
        half = 1;
        self->ready_mask = ready & ~0x02;
    }
    MICROPY_END_ATOMIC_SECTION(atomic_state);
    return half;
}

uint8_t adc_stream_hw_available(adc_stream_hw_t *self) {
    mp_uint_t atomic_state = MICROPY_BEGIN_ATOMIC_SECTION();
    uint8_t ready = self->ready_mask;
    MICROPY_END_ATOMIC_SECTION(atomic_state);
    return (ready & 1) + ((ready >> 1) & 1);
}

uint32_t adc_stream_hw_overruns(adc_stream_hw_t *self) {
    mp_uint_t atomic_state = MICROPY_BEGIN_ATOMIC_SECTION();
    uint32_t value = self->overruns;
    MICROPY_END_ATOMIC_SECTION(atomic_state);
    return value;
}

uint32_t adc_stream_hw_error(adc_stream_hw_t *self) {
    mp_uint_t atomic_state = MICROPY_BEGIN_ATOMIC_SECTION();
    uint32_t value = self->error;
    if (self->adc.Instance == ADC1
        && __HAL_ADC_GET_FLAG(&self->adc, ADC_FLAG_OVR)) {
        value |= HAL_ADC_ERROR_OVR;
    }
    MICROPY_END_ATOMIC_SECTION(atomic_state);
    return value;
}

bool adc_stream_hw_running(adc_stream_hw_t *self) {
    return self->running;
}

void adc_stream_deinit_all(void) {
    adc_stream_hw_t *active = adc_stream_active;
    if (active != NULL) {
        adc_stream_hw_stop(active);
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *adc) {
    adc_stream_hw_t *self = adc_stream_active;
    if (self == NULL || adc != &self->adc) {
        return;
    }

    // DMA is now writing the second half. Any unconsumed second-half view
    // is no longer safe to return.
    if (self->ready_mask & 0x02) {
        self->ready_mask &= ~0x02;
        ++self->overruns;
    }
    self->ready_mask |= 0x01;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *adc) {
    adc_stream_hw_t *self = adc_stream_active;
    if (self == NULL || adc != &self->adc) {
        return;
    }

    // DMA has wrapped and is now writing the first half.
    if (self->ready_mask & 0x01) {
        self->ready_mask &= ~0x01;
        ++self->overruns;
    }
    self->ready_mask |= 0x02;
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *adc) {
    adc_stream_hw_t *self = adc_stream_active;
    if (self != NULL && adc == &self->adc) {
        self->error |= HAL_ADC_GetError(adc);
        self->running = false;
    }
}
