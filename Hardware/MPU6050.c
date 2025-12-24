#include "stm32f10x.h"                  // Device header
#include <math.h>

#include "IIC.h"
#include "MPU6050_Reg.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"


#define MPU6050_ADDRESS		0x68		//MPU6050的I2C从机地址

#define q30  1073741824.0f //用于归一化四元数

#define DEFAULT_MPU_HZ  (200) //DMP采样频率200Hz

uint8_t MPU6050_GetID(void);

float q0,q1,q2,q3;
	
short gyro[3], accel[3], sensors; //用于读取DMP_FIFO

static signed char gyro_orientation[9] = { 1, 0, 0,  //方向矩阵
                                           0, 1, 0,
                                           0, 0, 1};

static  unsigned short inv_row_2_scale(const signed char *row) //用于初始化DMP
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;

}


static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx) //用于初始化DMP
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;

}

static void run_self_test1(void) //用于初始化DMP
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
//        Usart_Printf("setting bias succesfully ......\r\n");
    }

}

// DMP（数字运动处理器）初始化函数
int DMP_Init(void)
{
    int ret;
    uint8_t id = MPU6050_GetID();

    // 允许 0x68/0x69(MPU6050) 以及 0x70/0x71(常见于6500系列)
    if (id != 0x68 && id != 0x69 && id != 0x70 && id != 0x71)
        return -100;   // ID不对

    // 1) mpu_init
    ret = mpu_init();
    if (ret) return ret;  // 或者 return ret; 看你想不想保留原始码

    // 2) 使能传感器
    ret = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if (ret) return -2;

    // 3) 配置FIFO
    ret = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if (ret) return -3;

    // 4) 设置采样率
    ret = mpu_set_sample_rate(DEFAULT_MPU_HZ);
    if (ret) return -4;

    // 5) 加载DMP固件（最常见卡/失败点）
    ret = dmp_load_motion_driver_firmware();
    if (ret) return -5;

    // 6) 设置方向矩阵
    ret = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    if (ret) return -6;

    // 7) 使能DMP特性
    ret = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                             DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
                             DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
    if (ret) return -7;

    // 8) 设置DMP输出速率
    ret = dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    if (ret) return -8;

    // 9) 自检（这里一般不建议失败就死；你可以选择忽略或返回）
    run_self_test1();

    // 10) 打开DMP
    ret = mpu_set_dmp_state(1);
    if (ret) return -9;

    return 0; // 成功
}

u8 MPU6050_DMP_Get_Data(float *pitch, float *roll, float *yaw)
{
    unsigned long sensor_timestamp;
    unsigned char more;
    long quat[4];
    
    // 从DMP FIFO读取四元数数据
    if (dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more))
        return 1;

    // 四元数数据解析（q30格式转浮点）
    if (sensors & INV_WXYZ_QUAT) 
    {
         q0 = quat[0] / q30;
         q1 = quat[1] / q30;
         q2 = quat[2] / q30;
         q3 = quat[3] / q30;

        // 欧拉角计算（单位：度）
        *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;  // 俯仰角
        *roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, 
                      -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;  // 横滚角
        *yaw   = atan2(2 * (q1 * q2 + q0 * q3),
                       q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;  // 偏航角
    }
    else {
        return 2;
    }
    
    return 0;
}


/**
  * 函    数：MPU6050写寄存器  从机地址-寄存器地址-要写入的数据
  * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
  * 参    数：Data 要写入寄存器的数据，范围：0x00~0xFF
  * 返 回 值：无
  */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    IIC_Start();						//I2C起始
    IIC_SendByte(MPU6050_ADDRESS);	//发送从机地址，读写位为0，表示即将写入
    IIC_ReceiveAck();					//接收应答
    IIC_SendByte(RegAddress);			//发送寄存器地址
    IIC_ReceiveAck();					//接收应答
    IIC_SendByte(Data);				//发送要写入的数据
    IIC_ReceiveAck();					//接收应答
    IIC_Stop();						//I2C终止
}

/**
  * 函    数：MPU6050读寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
  * 返 回 值：读取寄存器的数据，范围：0x00~0xFF
  */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	IIC_Start();						//I2C起始
	IIC_SendByte(MPU6050_ADDRESS<<1);	//发送从机地址，读写位为0，表示即将写入
	IIC_ReceiveAck();					//接收应答
	IIC_SendByte(RegAddress);			//发送寄存器地址
	IIC_ReceiveAck();					//接收应答
	
	IIC_Start();						//I2C重复起始
	IIC_SendByte(MPU6050_ADDRESS<<1 | 0x01);	//发送从机地址，读写位为1，表示即将读取
	IIC_ReceiveAck();					//接收应答
	Data = IIC_ReceiveByte();			//接收指定寄存器的数据
	IIC_SendAck(1);					//发送应答，给从机非应答，终止从机的数据输出
	IIC_Stop();						//I2C终止
	
	return Data;
}

/**
  * 函    数：MPU6050初始化
  * 参    数：无
  * 返 回 值：无
  */
void MPU6050_Init(void)
{
	IIC_Init();									//先初始化底层的I2C
	
	/*MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器*/
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);		//电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);		//电源管理寄存器2，保持默认值0，所有轴均不待机
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);		//采样率分频寄存器，配置采样率
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);			//配置寄存器，配置DLPF
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);	//陀螺仪配置寄存器，选择满量程为±2000°/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);	//加速度计配置寄存器，选择满量程为±16g
}

/**
  * 函    数：MPU6050获取ID号
  * 参    数：无
  * 返 回 值：MPU6050的ID号
  */
uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);		//返回WHO_AM_I寄存器的值
}

/**
  * 函    数：MPU6050获取数据
  * 参    数：AccX AccY AccZ 加速度计X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
  * 参    数：GyroX GyroY GyroZ 陀螺仪X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
  * 返 回 值：无
  */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t DataH, DataL;								//定义数据高8位和低8位的变量
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);		//读取加速度计X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);		//读取加速度计X轴的低8位数据
	*AccX = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);		//读取加速度计Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);		//读取加速度计Y轴的低8位数据
	*AccY = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);		//读取加速度计Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);		//读取加速度计Z轴的低8位数据
	*AccZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);		//读取陀螺仪X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);		//读取陀螺仪X轴的低8位数据
	*GyroX = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);		//读取陀螺仪Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);		//读取陀螺仪Y轴的低8位数据
	*GyroY = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);		//读取陀螺仪Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);		//读取陀螺仪Z轴的低8位数据
	*GyroZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
}

