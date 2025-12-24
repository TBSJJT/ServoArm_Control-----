#include "FILTER.h"
#include <stdint.h>
#include <string.h>


void MPU_FILTER_Init(MPU_FILTER *f)//f 是指向 MovAvgF_t 结构体的指针 即把结构体清零
{
	memset (f,0,sizeof(MPU_FILTER));
}

float MPU_FILTER_Update(MPU_FILTER *f, float x)
{
	if(f->count < window)//样本小于容量
	{
		f->Buf[f->index] = x;//写入数据
		f->sum += x;//累加数据
		f->index = (uint8_t)((f->index + 1) % window);//更新指针
		f->count ++;
	}else
	{
		f->sum -= f->Buf[f->index];//减掉第一个数
		f->Buf[f->index] = x;           // 覆盖最老值
		f->sum += x;
		 f->index = (uint8_t)((f->index + 1) % window);//更新指针
	}
	
	return (f->sum / (float)f->count);//返回平均值
}
