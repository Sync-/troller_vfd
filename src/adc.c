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
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>

static void usart_setup(void)
{
	/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART2);

	/* Setup GPIO pin GPIO_USART1_TX/GPIO9 on GPIO port A for transmit. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);

	/* Setup UART parameters. */
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART2);
}

static void gpio_setup(void)
{
	/* Enable GPIO clocks. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOC);

	/* Setup the LEDs. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO15);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO1);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO2);

   gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
            GPIO_CNF_INPUT_ANALOG, GPIO0 | GPIO1 | GPIO4 | GPIO5 | GPIO6 | GPIO7 );  
   gpio_set_mode(GPIOC, GPIO_MODE_INPUT,
            GPIO_CNF_INPUT_ANALOG, GPIO8 );  
}

volatile uint16_t ADC_values[16];
volatile uint32_t status = 0;

static void dma_setup(void) {
   rcc_periph_clock_enable(RCC_DMA1);

   dma_channel_reset(DMA1, DMA_CHANNEL1);

   dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&ADC1_DR);  
   dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)ADC_values);
   dma_set_number_of_data(DMA1, DMA_CHANNEL1, 7);
   dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
   dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL1);
   dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
   dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
   dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
   dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
   dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_HIGH);

   dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
   nvic_set_priority(NVIC_DMA1_CHANNEL1_IRQ, 0);
   nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
  
   dma_enable_channel(DMA1, DMA_CHANNEL1); 
}

static void adc_setup(void)
{
	int i;

	rcc_periph_clock_enable(RCC_ADC1);

	uint8_t channel_array[16];
	/* Make sure the ADC doesn't run during config. */
	adc_off(ADC1);

	/* We configure everything for one single conversion. */
	adc_enable_scan_mode(ADC1);
//	adc_set_continuous_conversion_mode(ADC1);
   adc_set_single_conversion_mode(ADC1);
	adc_disable_external_trigger_regular(ADC1);
//   adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_SWSTART);
	adc_set_right_aligned(ADC1);
	/* We want to read the temperature sensor, so we have to enable it. */
	adc_enable_temperature_sensor(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);

	channel_array[0] = 0;
	channel_array[1] = 1;
	channel_array[2] = 4;
	channel_array[3] = 5;
	channel_array[4] = 6;
	channel_array[5] = 7;
	channel_array[6] = 8;
	adc_set_regular_sequence(ADC1, 2, channel_array);
   adc_enable_dma(ADC1);

	adc_power_on(ADC1);

	/* Wait for ADC starting up. */
	for (i = 0; i < 800000; i++)    /* Wait a bit. */
		__asm__("nop");

	adc_reset_calibration(ADC1);
	adc_calibration(ADC1);
}


void dma1_channel1_isr(void) {
   if(DMA_ISR_TCIF(DMA_CHANNEL1)) {
      status = 1;
      dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_ISR_GIF(DMA_CHANNEL1));
   }
}

int main(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	gpio_setup();
	gpio_clear(GPIOC, GPIO0);	                /* LED1 on */
	usart_setup();
   dma_setup();
	adc_setup();


	gpio_set(GPIOC, GPIO15);		/* LED2 off */

	adc_start_conversion_regular(ADC1);
//      while(!status) {
//         __asm__("nop");
//      }
	gpio_set(GPIOC, GPIO0);	                /* LED1 on */
	/* Continously convert and poll the temperature ADC. */
	while (1) {

	for (uint32_t i = 0; i < 800000; i++)    /* Wait a bit. */
		__asm__("nop");
		gpio_toggle(GPIOC, GPIO1); /* LED2 on */

	adc_start_conversion_direct(ADC1);
	}

	return 0;
}
