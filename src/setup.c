#include "setup.h"

void gpio_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
   rcc_periph_clock_enable(RCC_AFIO);

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

	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO1);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO2);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO3);

   gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
            GPIO_CNF_INPUT_ANALOG, GPIO0 | GPIO1 
            | GPIO4 | GPIO5 | GPIO6 | GPIO7 );  
   gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
            GPIO_CNF_INPUT_ANALOG, GPIO0 );  

   //Timer output
   gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
            GPIO_TIM1_CH1 | GPIO_TIM1_CH2 | GPIO_TIM1_CH3);

}

void usart_setup(void)
{
	rcc_periph_clock_enable(RCC_USART2);

	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO2);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO3);

	usart_set_baudrate(USART2, 921600);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	usart_enable(USART2);

	nvic_set_priority(NVIC_DMA1_CHANNEL7_IRQ, 1);
	nvic_enable_irq(NVIC_DMA1_CHANNEL7_IRQ);

   nvic_set_priority(NVIC_DMA1_CHANNEL6_IRQ, 1);
   nvic_enable_irq(NVIC_DMA1_CHANNEL6_IRQ);
}

void adc_setup(void)
{
	int i;

	rcc_periph_clock_enable(RCC_ADC1);

	uint8_t channel_array[16];
	adc_off(ADC1);

	adc_enable_scan_mode(ADC1);
	adc_set_continuous_conversion_mode(ADC1);
//	adc_disable_external_trigger_regular(ADC1);
   adc_enable_external_trigger_regular(ADC1, ADC_CR2_JEXTSEL_TIM2_TRGO);
	adc_set_right_aligned(ADC1);
	adc_enable_temperature_sensor(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);

	adc_power_on(ADC1);

	/* Wait for ADC starting up. */
	for (i = 0; i < 800000; i++)    /* Wait a bit. */
		__asm__("nop");

	adc_reset_calibration(ADC1);
	adc_calibration(ADC1);

   adc_enable_dma(ADC1);

	channel_array[0] = 0;
	channel_array[1] = 1;
	channel_array[2] = 4;
	channel_array[3] = 5;
	channel_array[4] = 6;
	channel_array[5] = 7;
	channel_array[6] = 8;
   channel_array[7] = 16; //Temperature
   channel_array[8] = 17; //Vrefint
	adc_set_regular_sequence(ADC1, 9, channel_array);

}

void sys_tick_handler(void)
{
   tick_ms++;
}

void systick_setup(void) {
   systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
   systick_set_reload(8999); //1ms
   systick_interrupt_enable();
   nvic_set_priority(NVIC_SYSTICK_IRQ, 0);
}

void timer_setup(void) {

   rcc_periph_clock_enable(RCC_TIM1);

   timer_reset             (TIM1);
   timer_set_mode          (TIM1, TIM_CR1_CKD_CK_INT,
                        TIM_CR1_CMS_CENTER_3, TIM_CR1_DIR_UP);
   timer_set_prescaler     (TIM1, 8);
//   timer_set_period        (TIM1, 2000);
   timer_set_period        (TIM1,1999);
   timer_set_oc_mode       (TIM1, TIM_OC1, TIM_OCM_PWM1);
   timer_set_oc_mode       (TIM1, TIM_OC2, TIM_OCM_PWM1);
   timer_set_oc_mode       (TIM1, TIM_OC3, TIM_OCM_PWM1);
   timer_set_oc_polarity_low(TIM1, TIM_OC1);
   timer_set_oc_polarity_low(TIM1, TIM_OC2);
   timer_set_oc_polarity_low(TIM1, TIM_OC3);
   timer_enable_oc_output  (TIM1, TIM_OC1);
   timer_enable_oc_output  (TIM1, TIM_OC2);
   timer_enable_oc_output  (TIM1, TIM_OC3);
   timer_enable_break_main_output(TIM1);
   timer_set_oc_value      (TIM1, TIM_OC1, 2);
   timer_set_oc_value      (TIM1, TIM_OC2, 2);
   timer_set_oc_value      (TIM1, TIM_OC3, 2);
   timer_enable_preload    (TIM1);
   timer_enable_counter    (TIM1);


   rcc_periph_clock_enable(RCC_TIM2);

   timer_reset(TIM2);
   timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT,
                           TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
   timer_set_prescaler(TIM2, 8); //9MHz clkin
   timer_set_period(TIM2, 90);  //Min 10kHz
   timer_set_clock_division(TIM2, 0);
   timer_set_master_mode(TIM2, TIM_CR2_MMS_UPDATE);
   timer_enable_counter(TIM2);

   DBGMCU_CR |= DBGMCU_CR_TIM1_STOP;
   DBGMCU_CR |= DBGMCU_CR_TIM2_STOP;

}
