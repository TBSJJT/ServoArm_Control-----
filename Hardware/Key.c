#include "Key.h"

#define DEBOUNCE_MS 20
#define HOLD_MS     500

extern  volatile uint32_t systick_ms;

void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11  | GPIO_Pin_14 | GPIO_Pin_15;// | GPIO_Pin_12 | GPIO_Pin_13
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
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
    uint8_t level = (GPIO_ReadInputDataBit(key->port, key->pin) == 0); // 低电平=按下

    switch(key->state)
    {
        case KEY_RELEASED:
            if(level)
            {
                key->state = KEY_DEBOUNCE;
                key->timestamp = now;
            }
            break;

        case KEY_DEBOUNCE://消抖确认态
            if(now - key->timestamp >= DEBOUNCE_MS)
            {
                if(level)
                {
                    key->state = KEY_PRESSED;
                    key->timestamp = now;   // 记录“稳定按下”的起始时间
                    // ★ 不再 return 1（不在按下瞬间判短按）
                }
                else
                {
                    key->state = KEY_RELEASED;
                }
            }
            break;

        case KEY_PRESSED:
            if(level)
            {
                if(now - key->timestamp >= HOLD_MS)
                {
                    key->state = KEY_HOLD;
                    key->timestamp = now;
                    return 2; // 长按开始（只触发一次）
                }
            }
            else
            {
                // ★ 在松手时判定短按：按下时间 < HOLD_MS
                key->state = KEY_RELEASED;
                return 1; // 短按（只会在“未长按”的情况下触发）
            }
            break;

        case KEY_HOLD:
            if(level)
            {
                if(now - key->timestamp >= 100)
                {
                    key->timestamp = now;
                    return 3; // 长按持续触发
                }
            }
            else
            {
                key->state = KEY_RELEASED;
                // 可选：这里如果想要“长按松手事件”
                return 4;
            }
            break;
    }

    return 0;
}

