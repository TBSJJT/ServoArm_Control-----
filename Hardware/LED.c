#include "stm32f10x.h"                  // Device header
//灯光用于测试响应
void LED_Init(void)//LED初始化
{
	//打开GPIO口时钟，先打开复用才能修改是否停用复用功能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);
	//关闭JTAG，使能SWD
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_SetBits (GPIOB, GPIO_Pin_3 | GPIO_Pin_4);
}

void LED1_ON(void)
{
	GPIO_ResetBits(GPIOB ,GPIO_Pin_4);
}

void LED1_OFF(void)
{
	GPIO_SetBits(GPIOB ,GPIO_Pin_4);
}

void LED2_ON(void)
{
	GPIO_ResetBits(GPIOB ,GPIO_Pin_3);
}

void LED2_OFF(void)
{
	GPIO_SetBits(GPIOB ,GPIO_Pin_3);
}

