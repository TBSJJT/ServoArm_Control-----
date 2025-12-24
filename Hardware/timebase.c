#include "stm32f10x.h"
#include "timebase.h"

volatile uint32_t systick_ms = 0;  // 系统毫秒计数

// SysTick 中断，每 1ms 调用一次
void SysTick_Handler(void)
{
    systick_ms++;
}

// 初始化 SysTick 1ms
void Timebase_Init(void)
{
    SysTick_Config(SystemCoreClock / 1000);
}

// 返回当前毫秒数
uint32_t Millis(void)
{
    return systick_ms;
}
