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

volatile uint16_t cur_a = 0;
volatile uint16_t cur_b = 0;
volatile uint16_t cur_c = 0;
volatile uint8_t update = 0;
void tim1_up_isr(void) {
   if(timer_get_flag(TIM1, TIM_SR_UIF)) {
      timer_clear_flag(TIM1, TIM_SR_UIF);
//      gpio_toggle(GPIOC, GPIO1);
//      adc_inj = adc_read_injected(ADC1,1);
   }
}

void adc1_2_isr(void) {
   
   ADC_SR(ADC1) &= ~ADC_SR_JEOC;
   cur_a = adc_read_injected(ADC1,1);
   cur_b = adc_read_injected(ADC2,1);
//   cur_c = adc_read_injected(ADC1,3);
   update = 1;
         gpio_toggle(GPIOC, GPIO1);
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

 //  int32_t cur_a, cur_b, cur_c, cur_a_cal, cur_b_cal, cur_c_cal;
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


   uint32_t cur_a_cal, cur_b_cal, cur_c_cal;
   cur_a_cal = cur_a;
   cur_b_cal = cur_b;
   cur_c_cal = cur_c;

   int32_t cur_a_ma = 0;
   int32_t cur_b_ma = 0;
   int32_t cur_c_ma = 0;

   timer_set_oc_value(TIM1, TIM_OC1, 0);
   timer_set_oc_value(TIM1, TIM_OC2, 10);
   timer_set_oc_value(TIM1, TIM_OC3, 10);
//   timer_enable_irq(TIM1, TIM_DIER_UIE);

   uint16_t timer_val = 1000;

   while (1) {

            volt_dc = 158 * ADC_values[0];
            volt_a = 117 * ADC_values[1];
            volt_b = 117 * ADC_values[2];
            volt_c = 117 * ADC_values[3];

            if(update == 1) {      
               cur_a_ma = (cur_a - cur_a_cal)*30;
               cur_b_ma = (cur_b - cur_b_cal)*30;
//               cur_c_ma = (cur_c - cur_c_cal)*29;

               if(abs(cur_a_ma) > 1000) {
                  timer_val--;
               } else if(abs(cur_a_ma) < 900) {
                  timer_val++;
               }
               if(timer_val > 1900) {
                  timer_val = 1900;
               } else if(timer_val < 20) {
                  timer_val = 20;
               }
               timer_set_oc_value(TIM1, TIM_OC3, timer_val);
               update = 0;
            }
      if(cur_a > 1000 && cur_b > 1000 && cur_c > 1000) {
//         gpio_toggle(GPIOC, GPIO1);
      }
//            adc_inj = adc_read_injected(ADC1, 1);
   }
   return 0;
}
