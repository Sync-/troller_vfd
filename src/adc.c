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
#include <libopencm3/stm32/dbgmcu.h>
#include <math.h>

#include "globals.h"
#include "setup.h"
#include "dma.h"

void dma1_channel1_isr(void) {
   dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_ISR_TCIF(DMA_CHANNEL1));
}

int main(void)
{
   rcc_clock_setup_in_hse_8mhz_out_72mhz();
   systick_setup();
   gpio_setup();
   gpio_clear(GPIOC, GPIO0);
   usart_setup();
   dma_setup();
   adc_setup();
   timer_setup();

   DBGMCU_CR |= DBGMCU_CR_TIM1_STOP;
   DBGMCU_CR |= DBGMCU_CR_TIM2_STOP;

   //TODO: Needs check for the hardware running

   systick_counter_enable();

   uint32_t delay = 0,tick_before = 0;

   int32_t cur_a, cur_b, cur_c, cur_a_cal, cur_b_cal, cur_c_cal;
   uint32_t volt_dc, volt_a, volt_b, volt_c, temperature, vref;

   int16_t sine_deg = 0, pwm_a = 0, pwm_b = 0, pwm_c = 0;
   double sine_rad = 0;


   gpio_set(GPIOC, GPIO0);
   gpio_set(GPIOB, GPIO1);
   gpio_set(GPIOB, GPIO2);
   gpio_set(GPIOB, GPIO3);

   char text[10] = "test\r\n";

   //Current sensor zero cal

   for (uint32_t i = 0; i < 40000; i++)    /* Wait a bit. */
      __asm__("nop");
   double voltperbit = 1.2 / ADC_values[8];
   uint32_t bitspervolt = (uint32_t) 1.0 / voltperbit;
   double a_offset = (1.0 - (voltperbit * ADC_values[4]))*1000.0;
   double b_offset = (1.0 - (voltperbit * ADC_values[5]))*1000.0;
   double c_offset = (1.0 - (voltperbit * ADC_values[6]))*1000.0;
   uint8_t mv_per_amp = 66; //Could be used to factory cal sensors
   double current_res = voltperbit / (mv_per_amp/1000000.0);
   uint8_t current_res_int = (uint8_t) current_res;
   cur_a_cal = (int32_t) a_offset;
   cur_b_cal = (int32_t) b_offset;
   cur_c_cal = (int32_t) c_offset;
   cur_a = (bitspervolt - (ADC_values[4] + cur_a_cal))* current_res_int;

   uint16_t sine240 = 0, sine120 = 0;

   while (1) {

      volt_dc = 117 * ADC_values[0];
      volt_a = 117 * ADC_values[1];
      volt_b = 117 * ADC_values[2];
      volt_c = 117 * ADC_values[3];
    
   cur_a = (bitspervolt - (ADC_values[4] + cur_a_cal))* current_res_int;
   cur_c = (bitspervolt - (ADC_values[5] + cur_b_cal))* current_res_int;
   cur_b = (bitspervolt - (ADC_values[6] + cur_c_cal))* current_res_int;
      vref = ADC_values[8];   
      temperature = ADC_values[7];   
 
      tick_before = tick_ms;

      if(sine_deg > 360) {
         sine_deg = 0;
      }

      sine240 = sine_deg + 240;
      if(sine240 >= 360) {
         sine240 = sine240 - 360;
      }
      sine120 = sine_deg + 120;
      if(sine120 >= 360) {
         sine120 = sine120 - 360;
      }

      pwm_a = 1100 * sin((M_PI/180.0)*(sine_deg)); 
      pwm_b = 1100 * sin((M_PI/180.0)*(sine240)); 
      pwm_c = 1100 * sin((M_PI/180.0)*(sine120)); 

      if(pwm_a < pwm_b) {
         if(pwm_a < pwm_c) {
            pwm_b -= pwm_a;
            pwm_c -= pwm_a;
            pwm_a = 0;
         } else {
            pwm_a -= pwm_c;
            pwm_b -= pwm_c;
            pwm_c = 0;
         }
      }  else {
         if(pwm_b < pwm_c) {
            pwm_a -= pwm_b;
            pwm_c -= pwm_b;
            pwm_b = 0;
         } else {
            pwm_a -= pwm_c;
            pwm_b -= pwm_c;
            pwm_c = 0;
         }

      }
   
      pwm_a = 2000 - pwm_a;
      pwm_b = 2000 - pwm_b;
      pwm_c = 2000 - pwm_c;

      timer_set_oc_value(TIM1, TIM_OC1, pwm_a);
      timer_set_oc_value(TIM1, TIM_OC2, pwm_b);
      timer_set_oc_value(TIM1, TIM_OC3, pwm_c);
      
      sine_deg = sine_deg + 20;


      for (uint32_t i = 0; i < 8000; i++)    /* Wait a bit. */
         __asm__("nop");
      gpio_toggle(GPIOC, GPIO1);

      uint8_t foo[10] = {0};

      foo[0] = 0xff;
      foo[1] = (uint8_t) (volt_dc / 500);
      foo[2] = (uint8_t) (pwm_a / 10);
      foo[3] = (uint8_t) (pwm_b / 10);
      foo[4] = (uint8_t) (pwm_c / 10);
//      foo[5] = (uint8_t) (ADC_values[1]);
//      foo[6] = (uint8_t) (ADC_values[2]);
//      foo[7] = (uint8_t) (ADC_values[3]);
      foo[5] = (uint8_t) (cur_a/100) + 127;
      foo[6] = (uint8_t) (cur_b/100) + 127;
      foo[7] = (uint8_t) (cur_c/100) + 127;
//      foo[5] = 0x00;
//      foo[6] = 0x00;
//      foo[7] = 0x00;
      foo[8] = 0x80;

      transferred = 0;
      dma_write(foo, 9);

//      adc_start_conversion_direct(ADC1);
   }

   return 0;
}
