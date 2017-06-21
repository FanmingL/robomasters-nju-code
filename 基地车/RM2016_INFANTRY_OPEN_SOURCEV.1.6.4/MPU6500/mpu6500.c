#include "main.h"
#include "mpu6500.h"
#include "delay.h"

u8	mpu6500_buf[14];					//spi读取MPU6500后存放数据
u8 offset_flag = 0;						//校准模式标志位，为0未进行校准，为1进行校准
volatile MPU6050_RAW_DATA    MPU6050_Raw_Data;    //原始数据
volatile MPU6050_REAL_DATA   MPU6050_Real_Data;
AHRS ahrs;
S_INT16_XYZ	MPU6500_Acc_Offset	=	{0,0,0};		
S_INT16_XYZ	MPU6500_Gyro_Offset	=	{0,0,0};	
S_INT16_XYZ MPU6500_Acc = {0,0,0};
S_INT16_XYZ MPU6500_Gyro = {0,0,0};
int16_t MPU6050_FIFO[6][11] = {0};//[0]-[9]为最近10次数据 [10]为10次数据的平均值
int16_t HMC5883_FIFO[3][11] = {0};//[0]-[9]为最近10次数据 [10]为10次数据的平均值 注：磁传感器的采样频率慢，所以单独列出
float	mpu6500_tempreature = 0;
s16 mpu6500_tempreature_temp = 0;
s16 mpu6500_tempreature_Offset = 0;



void MPU6050_DataSave(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz) //[0]-[9]为最近10次数据 [10]为10次数据的平均值
{
	uint8_t i = 0;
	int32_t sum=0;
	
	for(i=1;i<10;i++)
	{
		MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
		MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
		MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
		MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
		MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
		MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
	}
	
	MPU6050_FIFO[0][9]=ax;//将新的数据放置到 数据的最后面
	MPU6050_FIFO[1][9]=ay;
	MPU6050_FIFO[2][9]=az;
	MPU6050_FIFO[3][9]=gx;
	MPU6050_FIFO[4][9]=gy;
	MPU6050_FIFO[5][9]=gz;
	
	for(i=0;i<10;i++)//求当前数组的合，再取平均值
	{	
		 sum+=MPU6050_FIFO[0][i];
	}
	MPU6050_FIFO[0][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[1][i];
	}
	MPU6050_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[2][i];
	}
	MPU6050_FIFO[2][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[3][i];
	}
	MPU6050_FIFO[3][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[4][i];
	}
	MPU6050_FIFO[4][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[5][i];
	}
	MPU6050_FIFO[5][10]=sum/10;
	
}

/*
 * 函数名：MPU6500_ReadValue
 * 描述  ：读取MPU6500原始数据
 * 输入  ：无
 * 输出  ：无
 */ 
#define MIN_GX 98
int16_t MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz
				,MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz;

void MPU6500_ReadValue(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
	uint8_t i;
	
	MPU6500_CS(0); 																	//使能SPI传输

	SPI5_Read_Write_Byte(ACCEL_XOUT_H|0x80); 				//从加速度计的寄存器开始进行读取陀螺仪和加速度计的值//发送读命令+寄存器号
	
	for(i	=	0;i	<	14;i++)														//一共读取14字节的数据
	{
		mpu6500_buf[i]	=	SPI5_Read_Write_Byte(0xff);	//输入0xff,因为slave不识别
	}	
	if(offset_flag == 0)
	{
		MPU6050_Lastax= BYTE16(s16, mpu6500_buf[0],  mpu6500_buf[1]) ;
		MPU6050_Lastay= BYTE16(s16, mpu6500_buf[2],  mpu6500_buf[3]) ;
		MPU6050_Lastaz = BYTE16(s16, mpu6500_buf[4],  mpu6500_buf[5]);
		MPU6050_Lastgx= BYTE16(s16, mpu6500_buf[8],  mpu6500_buf[9]);
		MPU6050_Lastgy= BYTE16(s16, mpu6500_buf[10],  mpu6500_buf[11]);
		MPU6050_Lastgz= BYTE16(s16, mpu6500_buf[12],  mpu6500_buf[13]);
		
		MPU6050_DataSave(MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz,MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz);  		
		*ax  =MPU6050_FIFO[0][10];
		*ay  =MPU6050_FIFO[1][10];
		*az = MPU6050_FIFO[2][10];
		*gx  =MPU6050_FIFO[3][10]-MPU6500_Gyro_Offset.X ;
		*gy = MPU6050_FIFO[4][10]-MPU6500_Gyro_Offset.Y ;
		*gz = MPU6050_FIFO[5][10]-MPU6500_Gyro_Offset.Z;
		
	//	*gx  =MPU6050_FIFO[3][10];
	//	*gy = MPU6050_FIFO[4][10];
	//	*gz = MPU6050_FIFO[5][10];
		
		mpu6500_tempreature_temp	=	BYTE16(s16, mpu6500_buf[6],  mpu6500_buf[7]);
		mpu6500_tempreature	=	(float)(35000+((521+mpu6500_tempreature_temp)*100)/34); // 原来分母为340，现在分子*100，即：扩大1000倍；
		mpu6500_tempreature = mpu6500_tempreature/1000;                             
		if(( -MIN_GX	<	*gx ) && (*gx < MIN_GX) ) *gx = 0;
		if(( -MIN_GX	<	*gy ) && (*gy < MIN_GX) ) *gy = 0;
		if(( -MIN_GX	<	*gz ) && (*gz < MIN_GX) ) *gz = 0;
	}
	else if(offset_flag)  //MPU6500处于校准模式
	{
		*ax= BYTE16(s16, mpu6500_buf[0],  mpu6500_buf[1]) ;
		*ay = BYTE16(s16, mpu6500_buf[2],  mpu6500_buf[3]);
		*az = BYTE16(s16, mpu6500_buf[4],  mpu6500_buf[5]);
		*gx = BYTE16(s16, mpu6500_buf[8],  mpu6500_buf[9]) ;
		*gy = BYTE16(s16, mpu6500_buf[10],  mpu6500_buf[11]) ;
		*gz = BYTE16(s16, mpu6500_buf[12],  mpu6500_buf[13]);
	}
	
	MPU6500_CS(1);  	    //禁止SPI传输
}

void MPU6050_getlastMotion6(int16_t* ax, int16_t* ay, 
		int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
	*ax  =MPU6050_FIFO[0][10];
	*ay  =MPU6050_FIFO[1][10];
	*az = MPU6050_FIFO[2][10];
	*gx  =MPU6050_FIFO[3][10]-MPU6500_Gyro_Offset.X;
	*gy = MPU6050_FIFO[4][10]-MPU6500_Gyro_Offset.Y;
	*gz = MPU6050_FIFO[5][10]-MPU6500_Gyro_Offset.Z;
	if(( -MIN_GX	<	*gx ) && (*gx < MIN_GX) ) *gx = 0;
	if(( -MIN_GX	<	*gy ) && (*gy < MIN_GX) ) *gy = 0;
	if(( -MIN_GX	<	*gz ) && (*gz < MIN_GX) ) *gz = 0;
	
}

static void MPU6500_EN(u8 *en_ax, u8 *en_ay, u8 *en_az, u8 *en_gx, u8 *en_gy, u8 *en_gz)
{
	u8 temp;
	MPU6500_CS(0); 
	SPI5_Read_Write_Byte(PWR_MGMT_2|0x80);
	
	temp = SPI5_Read_Write_Byte(0xff);
	*en_ax = temp&0x20;
	*en_ay = temp&0x10;
	*en_az = temp&0x08;
	*en_gx = temp&0x04;
	*en_gy = temp&0x02;
	*en_gz = temp&0x01;
	MPU6500_CS(1);
}

void MPU6500_CE(void)
{
	u8 en_ax=0,en_ay=0,en_az=0,en_gx=0,en_gy=0,en_gz=0;
	MPU6500_EN(&en_ax, &en_ay, &en_az, &en_gx, &en_gy, &en_gz);
	printf("ax:%d,ay:%d,az:%d,gx:%d,gy:%d,gz:%d\n",en_ax,en_ay,en_az,en_gx,en_gy,en_gz);
}

/*
 * 函数名：MPU6500_Date_Offset
 * 描述  ：MPU6500数据校准
 * 输入  ：校准次数
 * 输出  ：无
 */ 
void MPU6500_Date_Offset(u16 cnt)
{
	static S_INT32_XYZ Temp_Gyro , Temp_Acc;
	int i = 0;
	int16_t accgyroval[6];
	Temp_Gyro.X =	0;
	Temp_Gyro.Y =	0;
	Temp_Gyro.Z =	0;
	
	Temp_Acc.X = 0;
	Temp_Acc.Y = 0;
	Temp_Acc.Z = 0;
	
	offset_flag = 1;//进入MPU6500校准模式
	for(i = cnt; i > 0; i--)
	{
    MPU6500_ReadValue(&accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5]);
		
		Temp_Acc.X	+=	accgyroval[0];
		Temp_Acc.Y	+=	accgyroval[1];		
		//Temp_Acc.Z	+=	MPU6500_Acc.Z;
		Temp_Gyro.X	+=	accgyroval[3];
		Temp_Gyro.Y	+=	accgyroval[4];
		Temp_Gyro.Z	+=	accgyroval[5];

	}
	
	MPU6500_Acc_Offset.X 	=	Temp_Acc.X	/	cnt;
	MPU6500_Acc_Offset.Y 	=	Temp_Acc.Y	/	cnt;
	//MPU6500_Acc_Offset.Z  =	Temp_Acc.Z	/	cnt;	
	MPU6500_Gyro_Offset.X	= Temp_Gyro.X	/	cnt;
	MPU6500_Gyro_Offset.Y	= Temp_Gyro.Y	/	cnt;
	MPU6500_Gyro_Offset.Z =	Temp_Gyro.Z	/	cnt;

	offset_flag = 0;//退出MPU6500校准模式
}
/*
 * 函数名：MPU6500_Init
 * 描述  ：MPU6500初始化函数
 * 输入  ：无
 * 输出  ：0：初始化失败 1：初始化成功
 */ 
u8 MPU6500_Init(void)
{
	SPI5_Init();																//MPU6500 IO口和SPI初始化
	if( MPU6500_Read_Reg(WHO_AM_I)== 0x70)			//正确读取到6500的地址
	{		
		MPU6500_Write_Reg(PWR_MGMT_1,0X80);   		//电源管理,复位MPU6500
		delay_ms(100);
		MPU6500_Write_Reg(SIGNAL_PATH_RESET,0X07);//陀螺仪、加速度计、温度计复位
		delay_ms(100);
		MPU6500_Write_Reg(PWR_MGMT_1,0X03);   		//选择时钟源
		MPU6500_Write_Reg(PWR_MGMT_2,0X00);   		//使能加速度计和陀螺仪
		MPU6500_Write_Reg(CONFIG,0X02);						//低通滤波器 0x02 92hz (3.9ms delay) fs=1khz
		MPU6500_Write_Reg(SMPLRT_DIV,0X00);				//采样率1000/(1+0)=1000HZ
		MPU6500_Write_Reg(GYRO_CONFIG,0X10);  		//陀螺仪测量范围 0X10 正负1000度
		MPU6500_Write_Reg(ACCEL_CONFIG,0x00); 		//加速度计测量范围 0X00 正负8g
		MPU6500_Write_Reg(ACCEL_CONFIG2,0x02);		//加速度计速率1khz 滤波器460hz (1.94ms delay)
		MPU6500_Write_Reg(USER_CTRL	,0x20);
			
		return 1;
	}
	else return 0;
}




