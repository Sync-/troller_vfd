#pragma once
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dbgmcu.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>

#include "globals.h"

void gpio_setup(void);
void usart_setup(void);
void adc_setup(void);
void systick_setup(void);
void timer_setup(void);
