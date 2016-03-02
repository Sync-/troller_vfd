#pragma once
#include <libopencm3/stm32/timer.h>

#include "globals.h"

int32_t isin_S3(int32_t x);
void setpwm(int32_t sine_deg, int32_t volt_requested, uint32_t volt_dc, uint8_t phases);
void dcbrake(uint8_t brake_percent);
void stop(void;
void disable(void);
