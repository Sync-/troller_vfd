/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Thomas Otto <tommi@viadmin.org>
 * Copyright (C) 2012 Piotr Esden-Tempski <piotr@esden.net>
 * Copyright (C) 2012 Ken Sarkies <ksarkies@internode.on.net>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>

#include "globals.h"
#include "setup.h"
#include "dma.h"
#include "pwm.h"

void dma1_channel1_isr(void) {
   dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_ISR_TCIF(DMA_CHANNEL1));
}

void tim1_cc_isr(void) {
   if(timer_get_flag(TIM1, TIM_SR_UIF)) {
      gpio_toggle(GPIOC, GPIO1);
   }
}

int main(void)
{
   uint8_t rx_buffer[RX_BUFFER_SIZE];

   rcc_clock_setup_in_hse_8mhz_out_72mhz();
   systick_setup();
   gpio_setup();
   gpio_clear(GPIOC, GPIO0);
   usart_setup();
   dma_setup(&rx_buffer);
   adc_setup();
   timer_setup();

   //TODO: Needs check for the hardware running

   systick_counter_enable();

   uint32_t delay = 0,tick_before = 0;

   int32_t cur_a, cur_b, cur_c, cur_a_cal, cur_b_cal, cur_c_cal;
   uint32_t volt_dc, volt_a, volt_b, volt_c, temperature, vref;

   int32_t sine_deg = 0;

   gpio_set(GPIOC, GPIO0);
   gpio_set(GPIOB, GPIO1);
   gpio_set(GPIOB, GPIO2);
   gpio_set(GPIOB, GPIO3);

   char text[10] = "test\r\n";


   //Current sensor zero cal

   uint32_t adc_inj = 0;

   for (uint32_t i = 0; i < 40000; i++)    /* Wait a bit. */
      __asm__("nop");

   while (1) {

            volt_dc = 158 * ADC_values[0];
            volt_a = 117 * ADC_values[1];
            volt_b = 117 * ADC_values[2];
            volt_c = 117 * ADC_values[3];

            adc_inj = adc_read_injected(ADC1, 1);
   }
   return 0;
}
