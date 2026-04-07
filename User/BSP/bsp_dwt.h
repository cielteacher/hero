#ifndef __BSP_DWT_H
#define __BSP_DWT_H

#include "stdint.h"

void     DWT_Init(uint32_t cpu_mhz);
void     DWT_Update(void);

float    DWT_GetDeltaT(uint32_t *last_cnt);
double   DWT_GetDeltaT64(uint32_t *last_cnt);

uint64_t DWT_GetTime_us(void);
uint32_t DWT_GetTime_ms(void);
float    DWT_GetTime_s(void);

void     DWT_Delay_us(uint32_t us);
void     DWT_Delay_ms(uint32_t ms);

#endif


