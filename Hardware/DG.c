#include "stm32f10x.h"                  // Device header
#include "Delay.h"

void DG_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12| GPIO_Pin_15| GPIO_Pin_8| GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);   
}

uint8_t DG_Get(void)
{
	uint8_t DGNum;
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10) == 0 &&
	GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) == 0 &&
	GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) == 0)//三个对管都检测到则为有车经过 则DGNum置1否则置0
	{
		
		DGNum = 1;
	 }
	else if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == 0 &&
	GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == 0 &&
	GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9) == 0)
	{	
		DGNum = 2;
	}else
	{
		DGNum = 0;
	}
	return DGNum;
}
