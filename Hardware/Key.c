#include "Key.h"

#define DEBOUNCE_MS 20
#define HOLD_MS     500

extern  volatile uint32_t systick_ms;

void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14 | GPIO_Pin_15;//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// 安全时间差函数
static inline uint32_t safe_time_diff(uint32_t newer, uint32_t older)
{
    if (newer >= older) {
        return newer - older;  // 正常情况
    } else {
        // 发生溢出
        return (UINT32_MAX - older) + newer + 1;
    }
}

void Key_StructInit(Key_t* key, GPIO_TypeDef* port, uint16_t pin)
{
    key->port = port;
    key->pin = pin;
    key->state = KEY_RELEASED;
    key->timestamp = 0;
}

uint8_t Key_Scan(Key_t* key)
{
    uint32_t now = systick_ms;
    uint8_t level = GPIO_ReadInputDataBit(key->port, key->pin) == 0; // 低电平按下

    switch(key->state)
    {
        case KEY_RELEASED:
            if(level)
            {
                key->state = KEY_DEBOUNCE;
                key->timestamp = now;
            }
            break;

        case KEY_DEBOUNCE:
            if(safe_time_diff(now,key->timestamp) >= DEBOUNCE_MS)
            {
                if(level)
                {
                    key->state = KEY_PRESSED;
                    key->timestamp = now;
                    return 1; // 短按触发一次
                }
                else
                    key->state = KEY_RELEASED;
            }
            break;

        case KEY_PRESSED:
            if(level)
            {
                if(safe_time_diff(now,key->timestamp) >= HOLD_MS)
                {
                    key->state = KEY_HOLD;
                    key->timestamp = now;
                    return 2; // 长按第一次触发
                }
            }
            else
            {
                key->state = KEY_RELEASED;
            }
            break;

        case KEY_HOLD:
            if(level)
            {
                if(safe_time_diff(now,key->timestamp) >= 100) // 长按重复触发
                {
                    key->timestamp = now;
                    return 3; // 长按持续触发
                }
            }
            else
            {
                key->state = KEY_RELEASED;
            }
            break;
    }

    return 0; // 未触发
}

