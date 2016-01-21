#include "dma.h"


void dma_setup(void) {
   rcc_periph_clock_enable(RCC_DMA1);

   dma_channel_reset(DMA1, DMA_CHANNEL1);

   //source

   dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&ADC_DR(ADC1));  
   dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL1);
   dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);

   //dest
   dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)ADC_values);
   dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
   dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);

   //what and how much
   dma_set_number_of_data(DMA1, DMA_CHANNEL1, 7);
   dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);

   dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
   dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_HIGH);

   dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
  
   dma_enable_channel(DMA1, DMA_CHANNEL1); 

   nvic_set_priority(NVIC_DMA1_CHANNEL1_IRQ, 2);
   nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);

}

void dma_write(char *data, uint32_t size)
{
   dma_channel_reset(DMA1, DMA_CHANNEL7);

   dma_set_peripheral_address(DMA1, DMA_CHANNEL7, (uint32_t)&USART2_DR);
   dma_set_memory_address(DMA1, DMA_CHANNEL7, (uint32_t)data);
   dma_set_number_of_data(DMA1, DMA_CHANNEL7, size);
   dma_set_read_from_memory(DMA1, DMA_CHANNEL7);
   dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL7);
   dma_set_peripheral_size(DMA1, DMA_CHANNEL7, DMA_CCR_PSIZE_8BIT);
   dma_set_memory_size(DMA1, DMA_CHANNEL7, DMA_CCR_MSIZE_8BIT);
   dma_set_priority(DMA1, DMA_CHANNEL7, DMA_CCR_PL_VERY_HIGH);

   dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL7);

   dma_enable_channel(DMA1, DMA_CHANNEL7);

   usart_enable_tx_dma(USART2);
}

void dma1_channel7_isr(void)
{
	if ((DMA1_ISR &DMA_ISR_TCIF7) != 0) {
		DMA1_IFCR |= DMA_IFCR_CTCIF7;

		transferred = 1;
	}

	dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL7);

	usart_disable_tx_dma(USART2);

	dma_disable_channel(DMA1, DMA_CHANNEL7);
}

