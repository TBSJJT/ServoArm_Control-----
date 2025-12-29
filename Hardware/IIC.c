#include "IIC.h"

#define IIC_SCL_PIN        GPIO_Pin_6
#define IIC_SDA_PIN        GPIO_Pin_7
#define IIC_GPIO_PORT      GPIOB

void IIC_Init(void)//引脚配置
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = IIC_SCL_PIN | IIC_SDA_PIN; // SCL and SDA
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; // Open-drain output
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(IIC_GPIO_PORT, &GPIO_InitStructure);
    /*设置默认电平*/
	GPIO_SetBits(IIC_GPIO_PORT, IIC_SCL_PIN | IIC_SDA_PIN);			//设置引脚初始化后默认为高电平（释放总线状态）

}

void IIC_W_SDA(uint8_t bit)//写SDA
{
    GPIO_WriteBit(IIC_GPIO_PORT, IIC_SDA_PIN, (BitAction)bit);
    Delay_us(5);
}

void IIC_W_SCL(uint8_t bit)//写SCL
{
    GPIO_WriteBit(IIC_GPIO_PORT, IIC_SCL_PIN, (BitAction)bit);
    Delay_us(5);
}

uint8_t IIC_R_SDA(void)//读SDA
{
    uint8_t bit;
    bit = GPIO_ReadInputDataBit(IIC_GPIO_PORT, IIC_SDA_PIN);
    Delay_us(5);
    return bit;
}

void IIC_Start(void)//IIC起始信号
{
    IIC_W_SCL(1);
    IIC_W_SDA(1);
    IIC_W_SDA(0);
    IIC_W_SCL(0);
}

void IIC_Stop(void)//IIC停止信号
{
    IIC_W_SDA(0);
    IIC_W_SCL(1);
    IIC_W_SDA(1);
}

void IIC_SendByte(uint8_t byte)//IIC发送一个字节
{
    uint8_t i;
    for(i = 0; i < 8; i++)
    {
        IIC_W_SDA(!!(byte & (0x80 >> i)));//使用掩码的方式取出Byte的指定一位数据并写入到SDA线
        IIC_W_SCL(1);
        IIC_W_SCL(0);
    }
}

uint8_t IIC_ReceiveByte(void)//IIC接收一个字节
{
    uint8_t i,byte = 0x00;   
    IIC_W_SDA(1);   //释放SDA总线
    for( i = 0; i < 8; i++)
    {
        IIC_W_SCL(1);//释放SCL总线，主机在SCL的高电平期间读取数据
        if (IIC_R_SDA()){(byte |= (0x80 >> i));}
        IIC_W_SCL(0);//拉低SCL总线，从机在SCL低电平期间写数据
    }
    return byte;
}

void IIC_SendAck(uint8_t ack)//IIC发送应答
{
    IIC_W_SDA(ack);
    IIC_W_SCL(1);
    IIC_W_SCL(0);
}

uint8_t IIC_ReceiveAck(void)//IIC接收应答  返回 0 表示收到了 ACK
{
    uint8_t ack;
    IIC_W_SDA(1); //释放SDA总线，等待从机应答
    IIC_W_SCL(1);
    ack = IIC_R_SDA();
    IIC_W_SCL(0);
    return ack;
}

int MPU6050_Read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)// 向指定从机指定地址读len个字节，用于dmp
{
    IIC_Start();
    IIC_SendByte(addr << 1);
    IIC_ReceiveAck();
    IIC_SendByte(reg);
    IIC_ReceiveAck();
    IIC_Start();
    IIC_SendByte((addr << 1)+1);
    IIC_ReceiveAck();
    while (len) {
        if (len == 1)
		{
            *buf = IIC_ReceiveByte();
			IIC_SendAck(1);
		}
        else
		{
            *buf = IIC_ReceiveByte();
			IIC_SendAck(0);
		}
        buf++;
        len--;
    }
	IIC_Stop();
	return 0;
}

int MPU6050_Write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data) //向指定地址写len个字节,用于dmp
{
	int i;
    IIC_Start();
    IIC_SendByte(addr << 1);
    IIC_ReceiveAck();
    IIC_SendByte(reg);
    IIC_ReceiveAck();
	for (i = 0; i < len; i++) 
	{
        IIC_SendByte(data[i]);
        IIC_ReceiveAck();
    }
    IIC_Stop();
	return 0;
}

