#include "Delay.h"
#include <stdint.h>

/* 有些老 CMSIS 没有这些宏/结构体，自己补最小集合 */
#ifndef CoreDebug_DEMCR_TRCENA_Msk
#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24)
#endif

#ifndef DWT_BASE
#define DWT_BASE (0xE0001000UL)
#endif

typedef struct
{
  volatile uint32_t CTRL;     // 0x00
  volatile uint32_t CYCCNT;   // 0x04
  volatile uint32_t CPICNT;   // 0x08
  volatile uint32_t EXCCNT;   // 0x0C
  volatile uint32_t SLEEPCNT; // 0x10
  volatile uint32_t LSUCNT;   // 0x14
  volatile uint32_t FOLDCNT;  // 0x18
  volatile uint32_t PCSR;     // 0x1C
} DWT_Type;

#ifndef DWT
#define DWT ((DWT_Type *)DWT_BASE)
#endif

#ifndef DWT_CTRL_CYCCNTENA_Msk
#define DWT_CTRL_CYCCNTENA_Msk (1UL << 0)
#endif

void DWT_Delay_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 允许 DWT/ITM
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;           // 使能 CYCCNT
}

void Delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000UL);
  while ((uint32_t)(DWT->CYCCNT - start) < ticks) {;}
}


/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
} 
