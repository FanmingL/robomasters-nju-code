#ifndef __MPU6500_DRIVER_H
#define __MPU6500_DRIVER_H

#include "stm32f4xx.h"


#define MPU6500_CS(X)			(X==0)?GPIO_ResetBits(GPIOF,GPIO_Pin_6):GPIO_SetBits(GPIOF,GPIO_Pin_6) //MPU6500片选信号
typedef struct __MPU6050_RAW_Data__
{
    short Accel_X;  //寄存器原生X轴加速度表示值
    short Accel_Y;  //寄存器原生Y轴加速度表示值
    short Accel_Z;  //寄存器原生Z轴加速度表示值
    short Temp;     //寄存器原生温度表示值
    short Gyro_X;   //寄存器原生X轴陀螺仪表示值
    short Gyro_Y;   //寄存器原生Y轴陀螺仪表示值
    short Gyro_Z;   //寄存器原生Z轴陀螺仪表示值
	  short Mag_X;   //寄存器原生X轴陀螺仪表示值
    short Mag_Y;   //寄存器原生Y轴陀螺仪表示值
    short Mag_Z;   //寄存器原生Z轴陀螺仪表示值
	
}MPU6050_RAW_DATA;

typedef struct __MPU6050_REAL_Data__
{
    float Accel_X;  //转换成实际的X轴加速度，
    float Accel_Y;  //转换成实际的Y轴加速度，
    float Accel_Z;  //转换成实际的Z轴加速度，
    float Temp;     //转换成实际的温度，单位为摄氏度
    float Gyro_X;   //转换成实际的X轴角加速度，
    float Gyro_Y;   //转换成实际的Y轴角加速度，
    float Gyro_Z;   //转换成实际的Z轴角加速度
	  float Mag_X;   //转换成实际的X轴角加速度，
    float Mag_Y;   //转换成实际的Y轴角加速度，
    float Mag_Z;   //转换成实际的Z轴角加速度
	
}MPU6050_REAL_DATA;

//define the eluer angle
typedef struct AHRS
{
	float pitch;
	float roll;
	float yaw;
	
}AHRS;
extern AHRS ahrs;

//the max and min data of the mag
typedef __packed struct
{
	int16_t MaxMagX;
	int16_t MaxMagY;
	int16_t MaxMagZ;
	int16_t MinMagX;
	int16_t MinMagY;
	int16_t MinMagZ;
}MagMaxMinData_t;
extern MagMaxMinData_t MagMaxMinData;

#define GYRO_CALI_FLAG 	  		(1<<0)
#define HMC5883_CALI_FLAG 		(1<<1)
#define ENCODER_CALI_FLAG 		(1<<2)
#define ALL_SENSOR_CALI_FLAG	(GYRO_CALI_FLAG|HMC5883_CALI_FLAG|ENCODER_CALI_FLAG)



void SPI5_Init(void);
 void IST8310_getRaw(int16_t *x,int16_t *y,int16_t *z);
 void IST8310_mgetValues(volatile float *arry);
void IST8310_getlastValues(int16_t *hx,int16_t *hy, int16_t *hz);
 uint8_t IST8310_Init(void);
u8 MPU6500_Write_Reg(uint8_t reg,uint8_t value);
u8 MPU6500_Read_Reg(uint8_t reg);
u8 SPI5_Read_Write_Byte(uint8_t TxData);//SPI总线读写一个字节


#endif


