#ifndef __STORE_H
#define __STORE_H

#include "stm32f10x.h"

#define STORE_PAGE_ADDR 0x08007C00  // 最后一页起始地址
#define STORE_MARK 	0xA5A5            // 标识位

extern uint16_t store_data[512];    // 映射 Flash 一页，512 半字 = 1 KB

// 初始化存储
void Store_Init(void);

// 保存所有数据到 Flash
void store_save(void);

// 清空存储（保留标识位）
void store_clear(void);

// 浮点数转换成两个半字
void float_to_halfwords(float val, uint16_t* low, uint16_t* high);

// 半字转换成浮点数
float halfwords_to_float(uint16_t low, uint16_t high);

#endif
