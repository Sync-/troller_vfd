#pragma once

#include <inttypes.h>


#ifndef M_PI
#define M_PI           3.14159265358979323846 //Yay! C99...
#endif

typedef enum {DISABLED = 0, STOPPED, RUNNING, RAMPING, DC_BRAKE, STOPPING} state_t;
typedef enum {UP, DOWN} direction_t;
enum {RX_BUFFER_SIZE = 100, TX_BUFFER_SIZE = 10};

extern volatile uint8_t tx_buffer[TX_BUFFER_SIZE];
extern volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
extern state_t state;
extern direction_t direction;
extern uint32_t transferred;
extern uint32_t received;
extern uint16_t ADC_values[];
extern uint64_t tick_ms;
