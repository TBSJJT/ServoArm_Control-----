#include "stm32f10x.h"
#include "Key.h"
#include "EXTI.h"
#include "OLED.h"
#include "Delay.h"
#include "NRF24L01.h"
#include "NRF24L01_define.h"
#include "ADC_control.h"
#include "LED.h"
#include "MPU6050.h"
#include "MPU6050_Reg.h"
#include "FILTER.h"

extern uint16_t AD_Value[3];					//定义用于存放AD转换结果的全局数组  PA1:电量  PA2:前后  PA3:左右
extern volatile uint32_t systick_ms;
volatile uint8_t mpu_data_ready = 0;

uint8_t NRFCONTROL_Val[32];//接收控制信息数组
uint8_t NRF_TXFlag;//发送成功标志位

uint8_t Control_Flag = 1;//模式选择

int16_t Target_Value1;
int16_t Target_Value2;

Key_t key_UP, key_DOWN, key_LEFT, key_RIGHT, key_MODE;
float pitch,roll,yaw;

float pitch_filt,roll_filt;

MPU_FILTER pitch_f,roll_f;


int main(void)
{	
	SystemInit();
	DWT_Delay_Init();
	
	MPU6050_Init();
  DMP_Init();
	MPU6050_EXTI_Init();

	LED_Init();
	Key_Init();	
	Key_StructInit(&key_UP, GPIOB, GPIO_Pin_12);
	Key_StructInit(&key_LEFT, GPIOB, GPIO_Pin_13);
	Key_StructInit(&key_RIGHT, GPIOB, GPIO_Pin_14);
	Key_StructInit(&key_DOWN, GPIOB, GPIO_Pin_15);
	Key_StructInit(&key_MODE, GPIOB, GPIO_Pin_11);

	PowerADC_Init();
	NRF24L01_Init();			
	

	OLED_Init();

	OLED_ShowString(0 ,10,"pitch:",OLED_8X16);
	OLED_ShowString(0 ,30,"roll:", OLED_8X16);
	OLED_Update();
	
	MPU_FILTER_Init(&pitch_f);
	MPU_FILTER_Init(&roll_f);
	while (1)
	{	

		if(Control_Flag == 0)
		{
		//电位器摇杆映射速度数据
			Target_Value1 = mapJoystickToAngle(AD_Value[1]);
			Target_Value2 = mapJoystickToAngle(AD_Value[2]);

			NRFCONTROL_Val[0] = (Target_Value1 >> 8) & 0xFF;
			NRFCONTROL_Val[1] = (Target_Value1)      & 0xFF;
			NRFCONTROL_Val[2] = (Target_Value2   >> 8) & 0xFF;
			NRFCONTROL_Val[3] = (Target_Value2)       & 0xFF;
		}else if(Control_Flag == 1)
		{
		//MPU6050陀螺仪映射速度数据
			if(mpu_data_ready == 1)
			{
				MPU6050_DMP_Get_Data(&pitch,&roll,&yaw);
				mpu_data_ready = 0;
				pitch_filt = MPU_FILTER_Update(&pitch_f, pitch);
				roll_filt  = MPU_FILTER_Update(&roll_f, roll);
			}
			Target_Value1 = (int16_t)((pitch_filt +50)/100*180);//-50  - 50 >> 0-180
			Target_Value2 = (int16_t)((-roll_filt +40)/80*180);//-40  - 40  >> 0-180
			if(Target_Value1 >  180) Target_Value1 =  180;
			if(Target_Value1 < 0) Target_Value1 = 0;
			if(Target_Value2 >  180) Target_Value2 =  180;
			if(Target_Value2 < 0) Target_Value2 = 0;

			NRFCONTROL_Val[0] = (Target_Value1 >> 8) & 0xFF;
			NRFCONTROL_Val[1] = (Target_Value1)      & 0xFF;
			NRFCONTROL_Val[2] = (Target_Value2   >> 8) & 0xFF;
			NRFCONTROL_Val[3] = (Target_Value2)       & 0xFF;
		}
		//速度显示
		OLED_ShowSignedNum(50,10,Target_Value1,3,OLED_8X16);
 		OLED_ShowSignedNum(50,30,Target_Value2,3,OLED_8X16);		
		OLED_Update();

		NRF_TXFlag = Send(NRFCONTROL_Val);//发送数据 
		
		if(NRF_TXFlag == TX_OK)//如果发送成功则对应的LED点亮
		{
			if(Target_Value1 != 0)LED1_ON();
			else LED1_OFF();
			if(Target_Value2 != 0)LED2_ON();
			else LED2_OFF();
		}else{LED1_OFF();LED2_OFF();}

	}
}

/*
void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
				
    }
}
*/

void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line10) != RESET)
	{		
		EXTI_ClearITPendingBit(EXTI_Line10);
		mpu_data_ready = 1;
	}

}
