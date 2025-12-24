#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"

typedef enum {
    KEY_RELEASED = 0,
    KEY_DEBOUNCE,
    KEY_PRESSED,
    KEY_HOLD
} KeyState_t;

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    KeyState_t state;
    uint32_t timestamp;
} Key_t;

void Key_Init(void);
void Key_StructInit(Key_t* key, GPIO_TypeDef* port, uint16_t pin);
uint8_t Key_Scan(Key_t* key);
uint8_t Key_IsPressed(Key_t* key);

#endif
