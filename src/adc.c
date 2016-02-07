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
#include <math.h>

#include "globals.h"
#include "setup.h"
#include "dma.h"
#include "pwm.h"

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

   for (uint32_t i = 0; i < 40000; i++)    /* Wait a bit. */
      __asm__("nop");
/*   float voltperbit = 1.2 / ADC_values[8];
   uint32_t bitspervolt = (uint32_t) 1.0 / voltperbit;
   float a_offset = (1.0 - (voltperbit * ADC_values[4]))*1000.0;
   float b_offset = (1.0 - (voltperbit * ADC_values[5]))*1000.0;
   float c_offset = (1.0 - (voltperbit * ADC_values[6]))*1000.0;
   uint8_t mv_per_amp = 66; //Could be used to factory cal sensors
   float current_res = voltperbit / (mv_per_amp/1000000.0);
   uint8_t current_res_int = (uint8_t) current_res;
   cur_a_cal = (int32_t) a_offset;
   cur_b_cal = (int32_t) b_offset;
   cur_c_cal = (int32_t) c_offset;
   //   cur_a = (bitspervolt - (ADC_values[4] + cur_a_cal))* current_res_int;
*/
   int32_t angle_per_tick = 14;

   int16_t frequency = 5, ramp_start = 5, ramp_end = 50;
   int16_t delta = (ramp_end - ramp_start);   

   static uint16_t ticks_per_s = 500;

   angle_per_tick = (int32_t) (frequency * 360) / ticks_per_s;

   uint32_t ramptime_ms = 10000;

   int16_t delta_mul;

   float ramptimemul;

   frequency = ramp_start;

   state = RUNNING;

   uint64_t brake_first = 0;
   uint64_t braketime = 5000;

   int32_t pwm_factor = 0;
   int32_t volt_requested = 0;

   while (1) {
      if(tick_ms > tick_before) {

         tick_before = tick_ms;

         angle_per_tick = (int32_t) (frequency * 360) / ticks_per_s;

         if(tick_ms <= ramptime_ms) {
            ramptimemul = tick_ms/(float)ramptime_ms;
            delta_mul = (delta * ramptimemul);

            frequency = ramp_start + delta_mul;
         }
         if(tick_ms%2 == 0 ) {
            volt_dc = 117 * ADC_values[0];
            volt_a = 117 * ADC_values[1];
            volt_b = 117 * ADC_values[2];
            volt_c = 117 * ADC_values[3];

/*            cur_a = (bitspervolt - (ADC_values[4] + cur_a_cal))* current_res_int;
            cur_c = (bitspervolt - (ADC_values[5] + cur_b_cal))* current_res_int;
            cur_b = (bitspervolt - (ADC_values[6] + cur_c_cal))* current_res_int;
             vref = ADC_values[8];   
            temperature = ADC_values[7]; */  
            if(state == RAMPING) {

            }
            if(state == RUNNING || state == RAMPING) {
               if(sine_deg > 360) {
                  sine_deg = 0;
               }

               volt_requested = frequency * 460;

               setpwm(sine_deg, volt_requested, volt_dc,3);

               sine_deg = sine_deg + angle_per_tick;

            } else {
               if(state == DC_BRAKE) {
                  brake_first = tick_ms;
                  dcbrake(50);
                  state = STOPPED;
               }
               if (state == STOPPED && tick_ms - brake_first > braketime) {
                  stop();
               }
            }

            tx_buffer[0] = 0xff;
            tx_buffer[1] = 0xff;
            tx_buffer[2] = 0xff;
            tx_buffer[4] = (uint8_t)pwm_factor;
            //            tx_buffer[1] = (uint8_t) (volt_dc / 500);
            //            tx_buffer[2] = (uint8_t) (pwm.pwm_a / 10);
            //            tx_buffer[3] = (uint8_t) (pwm.pwm_b / 10);
            //            tx_buffer[4] = (uint8_t) (pwm.pwm_c / 10);
            //      tx_buffer[5] = (uint8_t) (ADC_values[1]);
            //      tx_buffer[6] = (uint8_t) (ADC_values[2]);
            //      tx_buffer[7] = (uint8_t) (ADC_values[3]);
            tx_buffer[5] = (uint8_t) (cur_a/100) + 127;
            tx_buffer[6] = (uint8_t) (cur_b/100) + 127;
            tx_buffer[7] = (uint8_t) (cur_c/100) + 127;
            //      tx_buffer[5] = 0x00;
            //      tx_buffer[6] = 0x00;
            //      tx_buffer[7] = 0x00;
            tx_buffer[8] = 0x80;

            transferred = 0;
            dma_write(tx_buffer, 9);
         }
      }
   }
   return 0;
}
