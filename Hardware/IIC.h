#ifndef IIC_H
#define IIC_H

#include "stm32f10x.h"
#include "Delay.h"

void IIC_Init(void);//引脚配置
void IIC_W_SDA(uint8_t bit);//写SDA
uint8_t IIC_R_SDA(void);//读SDA
void IIC_W_SCL(uint8_t bit);//写SCL
void IIC_Start(void);//IIC起始信号
void IIC_Stop(void);//IIC停止信号
void IIC_SendByte(uint8_t byte);//IIC发送一个字节
uint8_t IIC_ReceiveByte(void);//IIC接收一个字节
void IIC_SendAck(uint8_t ack);//IIC发送应答
uint8_t IIC_ReceiveAck(void);//IIC接收应答

int MPU6050_Read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);// 向指定从机指定地址读len个字节，用于dmp
int MPU6050_Write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data); //向指定地址写len个字节,用于dmp


#endif
