#ifndef ADC_CONTROL_H
#define ADC_CONTROL_H
#include "stm32f10x.h"

#define ADC_MIN 0
#define ADC_MID 2048
#define ADC_MAX 4095
#define DEAD    50


void PowerADC_Init(void);
uint16_t Map_Pote(uint16_t adc);

#endif
