#pragma once

#include <inttypes.h>


#ifndef M_PI
#define M_PI           3.14159265358979323846 //Yay! C99...
#endif

typedef enum {DISABLED = 0, STOPPED, RUNNING, RAMPING, DC_BRAKE, STOPPING} state_t;
typedef enum {UP, DOWN} direction_t;

volatile extern uint8_t tx_buffer[];
volatile extern uint8_t rx_buffer[];
extern state_t state;
extern direction_t direction;
extern uint32_t transferred;
extern uint32_t received;
extern uint16_t ADC_values[];
extern uint64_t tick_ms;
