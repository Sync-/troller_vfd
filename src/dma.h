#pragma once

#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#include "globals.h"

void dma_setup(uint32_t *rx_buffer);
void dma_write(char *data, uint32_t size);
void dma_read(char *data, uint32_t size);
