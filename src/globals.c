#include "globals.h"

volatile uint8_t tx_buffer[TX_BUFFER_SIZE];
volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
state_t state = DISABLED;
direction_t direction = UP;
uint32_t transferred = 0;
uint32_t received = 0;
uint16_t ADC_values[16] = {0};
uint64_t tick_ms = 0;
