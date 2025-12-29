#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "Key.h"
#include "Serial.h"
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

#define VOFA_OPEN_JFloat 0
#if VOFA_OPEN_JFloat == 1

float data[4]; 
uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};

#endif

extern uint16_t AD_Value[5];					//定义用于存放AD转换结果的全局数组  PA1:电量  PA2:前后  PA3:左右
extern volatile uint32_t systick_ms;			//系统滴答计数器 毫秒	在#include "stm32f10x_it.c"
volatile uint8_t mpu_data_ready = 0;			//MPU6050数据准备好标志位

uint8_t NRFCONTROL_Val[32];	   //1*32个字节 
uint8_t NRF_TXFlag;	//发送成功标志位
//模式选择标志位
uint8_t Control_Flag = 0;						
//电位器映射数据
int16_t Target_Value1;
int16_t Target_Value2;
int16_t Target_Value3;
int16_t Target_Value4;
//定义按键结构体变量 
Key_t key_UP, key_DOWN, key_LEFT, key_RIGHT, key_MODE;
//滤波结构体变量			
MPU_FILTER_t pitch_f,roll_f,voltage_f;									
float pitch,roll,yaw,voltage_Value;//原始数据										
float pitch_filt,roll_filt,voltage_filt;//滤波后数据								
float Target_Value1_f,Target_Value2_f;//暂存发送的浮点目标值				



int main(void)
{	
	SystemInit();
	SysTick_Config(SystemCoreClock / 1000);//配置systick，systick中断1ms用于非阻塞状态机按键扫描
	SystemCoreClockUpdate();
	
	DWT_Delay_Init();//DWT延时
	
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
	
	Serial_Init(9600);

	OLED_Init();
	MPU_FILTER_Init(&pitch_f);
	MPU_FILTER_Init(&roll_f);
	MPU_FILTER_Init(&voltage_f);

	/*NVIC中断分组*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	while (1)
	{	
		if(Key_Scan(&key_MODE))//检测到按键变化
		{
			if(key_MODE.state == KEY_PRESSED)//检测到短按
			{
				Control_Flag ^= 1;//按位异或 1^1=0 0^1=1 切换模式
			}
		}
		//电源电压范围：7.4-8.4v 分压电阻：30k和10k ADC映射电量
		voltage_Value = AD_Value[4] ; // 电压值计算
		voltage_filt = MPU_FILTER_Update(&voltage_f, voltage_Value);
		OLED_ShowString(0,45,"Volt:",OLED_8X16);
		OLED_ShowFloatNum(40,45,voltage_filt* (30.0f + 10.0f) / 10.0f / 4095.0f * 3.3f,1,2,OLED_8X16);
		OLED_Update();
		if(Control_Flag == 0)
		{
		//电位器摇杆映射速度数据
			Target_Value1 = Map_Pote(AD_Value[0]);//右摇杆左右         4自由度舵机
			Target_Value2 = Map_Pote(AD_Value[1]);//右摇杆前后
			Target_Value3 = Map_Pote(AD_Value[2]);//左摇杆左右
			Target_Value4 = Map_Pote(AD_Value[3]);//左摇杆前后

			NRFCONTROL_Val[0] = 0xAA; //帧头
			NRFCONTROL_Val[1] = (Target_Value1 >> 8) & 0xFF;
			NRFCONTROL_Val[2] = (Target_Value1)      & 0xFF;
			NRFCONTROL_Val[3] = (Target_Value2 >> 8) & 0xFF;
			NRFCONTROL_Val[4] = (Target_Value2)      & 0xFF;
			
			NRFCONTROL_Val[5] = (Target_Value3 >> 8) & 0xFF;
			NRFCONTROL_Val[6] = (Target_Value3)      & 0xFF;
			NRFCONTROL_Val[7] = (Target_Value4 >> 8) & 0xFF;
			NRFCONTROL_Val[8] = (Target_Value4)      & 0xFF;
			
			//显示		
			OLED_ShowString(0 ,0,"Potentiometer ",OLED_8X16);//Potentiometer：电位器
			OLED_ShowString( 0,15,"Val1:",OLED_8X16);
			OLED_ShowString( 0,30,"Val2:",OLED_8X16);
			OLED_ShowString(68,15,"Val3:",OLED_8X16);
			OLED_ShowString(68,30,"Val4:",OLED_8X16);
			OLED_ShowNum(40,15,Target_Value1,3,OLED_8X16);
			OLED_ShowNum(40,30,Target_Value2,3,OLED_8X16);		
			OLED_ShowNum(106,15,Target_Value3,3,OLED_8X16);
			OLED_ShowNum(106,30,Target_Value4,3,OLED_8X16);		

			OLED_Update();
			
			NRF_TXFlag = Send(NRFCONTROL_Val);//发送数据 		
			if(NRF_TXFlag == TX_OK)//如果发送成功则对应的LED点亮
			{LED1_ON();LED2_ON();}else{LED1_OFF();LED2_OFF();}
		}
		else if(Control_Flag == 1)
		{
			//MPU6050陀螺仪滤波速度数据															2自由度
			if(mpu_data_ready == 1)
			{
				MPU6050_DMP_Get_Data(&pitch,&roll,&yaw);		
				mpu_data_ready = 0;
				pitch_filt = MPU_FILTER_Update(&pitch_f, pitch);
				roll_filt  = MPU_FILTER_Update(&roll_f, roll);			
#if VOFA_OPEN_JFloat == 1

			 //发送浮点数据给vofa+ (justfloat)
			data[0] = (pitch + 50.0f) * 180.0f / 100.0f;
			data[1] = (pitch_filt + 50.0f) * 180.0f / 100.0f;
			data[2] = (-roll + 40.0f) * 180.0f / 80.0f;
			data[3] = (-roll_filt + 40.0f) * 180.0f / 80.0f;
			Serial_SendArray((uint8_t *)data, sizeof(float) * 4); 
			// 发送帧尾
			Serial_SendArray(tail, 4);	
#endif
			}
			//映射目标值 0-180度
			Target_Value1_f = (pitch_filt + 50.0f) * 180.0f / 100.0f;     
			Target_Value2_f = (-roll_filt + 40.0f) * 180.0f / 80.0f;     
			// 限幅（float）
			if (Target_Value1_f > 180.0f) Target_Value1_f = 180.0f;
			if (Target_Value1_f < 0.0f)   Target_Value1_f = 0.0f;
			if (Target_Value2_f > 180.0f) Target_Value2_f = 180.0f;
			if (Target_Value2_f < 0.0f)   Target_Value2_f = 0.0f;
			// 转换为int16_t发送
			Target_Value1 = (int16_t)(Target_Value1_f * 100); // 放大100倍发送
			Target_Value2 = (int16_t)(Target_Value2_f * 100);
			NRFCONTROL_Val[0] = 0x55; //帧头
			NRFCONTROL_Val[1] = (Target_Value1 >> 8) & 0xFF;
			NRFCONTROL_Val[2] = (Target_Value1)      & 0xFF;
			NRFCONTROL_Val[3] = (Target_Value2 >> 8) & 0xFF;
			NRFCONTROL_Val[4] = (Target_Value2)      & 0xFF;
			
			NRF_TXFlag = Send(NRFCONTROL_Val);//发送数据
			if(NRF_TXFlag == TX_OK)//如果发送成功则对应的LED点亮
			{LED1_ON();LED2_ON();}else{LED1_OFF();LED2_OFF();}
			//显示		
			OLED_ShowString(0 ,0,"Gyroscope    ",OLED_8X16);//Gyroscope：陀螺仪
			OLED_ShowFloatNum(40,15,Target_Value1_f,3,2,OLED_8X16);
			OLED_ShowFloatNum(40,30,Target_Value2_f,3,2,OLED_8X16);		
			OLED_Update();

		}

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
