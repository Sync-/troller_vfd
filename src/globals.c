#include "globals.h"

uint8_t tx_buffer[10];
state_t state = DISABLED;
uint32_t transferred = 0;
uint16_t ADC_values[16] = {0};
uint64_t tick_ms = 0;
