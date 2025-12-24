#include "store.h"
#include "myflash.h"  // 你的 Flash 操作函数
#include <string.h>  // 用于 memcpy

uint16_t store_data[512];  // 映射 Flash 一页

// ------------------------------
// 浮点与半字转换
// ------------------------------
void float_to_halfwords(float val, uint16_t* low, uint16_t* high)
{
    uint32_t temp;
    memcpy(&temp, &val, 4);
    *low  = temp & 0xFFFF;
    *high = (temp >> 16) & 0xFFFF;
}

float halfwords_to_float(uint16_t low, uint16_t high)
{
    uint32_t temp = ((uint32_t)high << 16) | low;
    float val;
    memcpy(&val, &temp, 4);
    return val;
}

// ------------------------------
// 初始化存储
// ------------------------------
void Store_Init(void)
{
    uint16_t mark = MyFlash_ReadHalfWord(STORE_PAGE_ADDR);
    if(mark != STORE_MARK)
    {
        // 第一次使用，擦除 Flash 并初始化
        MyFlash_ERASEPAGE(STORE_PAGE_ADDR);
        MyFlash_ProgramHalfWord(STORE_PAGE_ADDR, STORE_MARK);
        for(uint16_t i = 1; i < 512; i++)
        {
            MyFlash_ProgramHalfWord(STORE_PAGE_ADDR + i*2, 0x0000);
        }
    }

    // 读取所有数据到 RAM
    for(uint16_t i = 0; i < 512; i++)
    {
        store_data[i] = MyFlash_ReadHalfWord(STORE_PAGE_ADDR + i*2);
    }
}

// ------------------------------
// 保存 Flash
// ------------------------------
void store_save(void)
{
    // 保存前必须擦除整页
    MyFlash_ERASEPAGE(STORE_PAGE_ADDR);
    for(uint16_t i = 0; i < 512; i++)
    {
        MyFlash_ProgramHalfWord(STORE_PAGE_ADDR + i*2, store_data[i]);
    }
}

// ------------------------------
// 清空 Flash (保留标识位)
// ------------------------------
void store_clear(void)
{
    for(uint16_t i = 1; i < 512; i++)
        store_data[i] = 0x0000;
    store_save();
}
