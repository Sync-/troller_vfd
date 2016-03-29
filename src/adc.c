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

   for (uint32_t i = 0; i < 40000; i++)    /* Wait a bit. */
      __asm__("nop");

   int32_t angle_per_tick = 14;
   int16_t frequency = 5, ramp_start = 5, ramp_end = 50;
   static uint16_t ticks_per_s = 500;

   angle_per_tick = (int32_t) (frequency * 360) / ticks_per_s;

   uint32_t ramptime_ms = 5000;
   uint64_t ramp_start_ms = 0;

   uint32_t slope = (ramp_end-ramp_start)*131072/ramptime_ms;

   frequency = ramp_start;

   state = RAMPING;
   direction = UP;

   uint64_t brake_first = 0;
   uint64_t braketime = 500;

   int32_t pwm_factor = 0;
   int32_t volt_requested = 0;

   uint8_t dcbrakeonstop = 1;

   uint32_t bufferpos = 0, bufferpos_prev = 0;
   uint32_t avalible = 0;
   uint32_t rxpos = 0;

   while (1) {

      bufferpos = 100 - DMA1_CNDTR6;

      avalible = (bufferpos - rxpos + 100)%100;
      if(bufferpos != bufferpos_prev) {
         bufferpos_prev = bufferpos;

         for(uint16_t i; i <= avalible; i++) {
            tx_buffer[i] = rx_buffer[bufferpos-avalible+rxpos];
            rxpos++;
         }

         transferred = 0;
         dma_write(tx_buffer, 9);
      }

      if(tick_ms > tick_before) {

         tick_before = tick_ms;

         angle_per_tick = (int32_t) (frequency * 360) / ticks_per_s;

         if(tick_ms == 20000) {
//            state = RAMPING;
//            direction = DOWN;
            state = STOPPING;
         }

         if(tick_ms%2 == 0 ) {
            volt_dc = 158 * ADC_values[0];
            volt_a = 117 * ADC_values[1];
            volt_b = 117 * ADC_values[2];
            volt_c = 117 * ADC_values[3];

            if(volt_dc > 150*1000) {
               state = DISABLED;
            }

            if(state == RAMPING || state == STOPPING) {
               if(ramp_start_ms == 0) {
                  ramp_start_ms = tick_ms;
                  gpio_toggle(GPIOC, GPIO1);
               }
               if(tick_ms < (ramp_start_ms + ramptime_ms)) {
                  if(direction == UP) {
                     frequency = ((slope * (tick_ms - ramp_start_ms))>>17)+ramp_start;
                     if(frequency > ramp_end) {
                        frequency = ramp_end;
                     }
                  } else {

                     frequency = (-1)*((slope*(tick_ms-ramp_start_ms))>>17)+ramp_end;
                     if(frequency < ramp_start) {
                        frequency = ramp_start;
                     }
                  }
               } else if(state != STOPPING){
                  state = RUNNING;
                  ramp_start_ms = 0;
                  gpio_toggle(GPIOC, GPIO1);
                  if(direction == UP) {
                     frequency = ramp_end;
                  } else {
                     frequency = ramp_start;
                  }
               } else {
                  if(dcbrakeonstop) {
                     state = DC_BRAKE;
                  } else {
                     state = STOPPED;
                  }
               }
            }
            if(state == RUNNING || state == RAMPING || state == STOPPING) {
               if(sine_deg > 360) {
                  sine_deg = 0;
               }
               if (state == STOPPING) {
                  direction = DOWN;
               }

               volt_requested = frequency * 1000; //4.6 -> 4600

               setpwm(sine_deg, volt_requested, volt_dc, 3);

               sine_deg = sine_deg + angle_per_tick;

            } else {
               if(state == DC_BRAKE) {
                  brake_first = tick_ms;
                  dcbrake(10);
                  state = STOPPED;
               } else if (state == STOPPED && tick_ms - brake_first > braketime) {
                  stop();
               } 
            }
            if(state == DISABLED) {
               disable();
            }
               tx_buffer[0] = 0xff;
               tx_buffer[1] = (uint8_t) (volt_dc/1000);
               tx_buffer[2] = (uint8_t) frequency;
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
