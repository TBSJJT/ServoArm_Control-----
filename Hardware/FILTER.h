#ifndef _FILTER_H
#define _FILTER_H

#include "stm32f10x.h"

#define window 10

typedef struct 
{
	uint8_t count;//有效的样本数
	uint8_t index;//下一次写入的位置
	float Buf[window];//缓冲区
	float sum;//样本和
}MPU_FILTER_t;

void MPU_FILTER_Init(MPU_FILTER_t *f);//f 是指向 MovAvgF_t 结构体的指针 即把结构体清零
float MPU_FILTER_Update(MPU_FILTER_t *f, float x);

#endif
