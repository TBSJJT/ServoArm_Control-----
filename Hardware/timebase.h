#ifndef TIMEBASE_H
#define TIMEBASE_H
#include <stdint.h>

void Timebase_Init(void);      // 初始化 SysTick
uint32_t Millis(void);         // 返回系统毫秒计时

#endif
