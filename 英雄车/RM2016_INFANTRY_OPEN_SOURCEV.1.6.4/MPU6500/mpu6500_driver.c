/*----SPI5_CS-----PF6----*/
/*----SPI_SCK-----PF7----*/
/*----SPI_MISO-----PF8----*/
/*----SPI_MOSI-----PF9----*/

#include "mpu6500_driver.h"
#include "main.h"
/*
 * ��������SPI1_Init
 * ����  ��SPI1��ʼ��
 * ����  ����
 * ���  ����
 */ 
 MagMaxMinData_t MagMaxMinData;
 float HMC5883_lastx,HMC5883_lasty,HMC5883_lastz;
 
 static void IST_Reg_Write_By_MPU(uint8_t addr, uint8_t data)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  Delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, addr);
  Delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, data);
  Delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x080 | 0x01);
  Delay_ms(10);
}
 
 static uint8_t IST_Reg_Read_By_MPU(uint8_t addr)
{
  uint8_t data;
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_REG, addr);
  delay_us(200);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x80);
  delay_us(200);
  data = MPU6500_Read_Reg(MPU6500_I2C_SLV4_DI);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
	delay_us(200);
  return data;
}
 static void MPU_Auto_Read_IST_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, device_address);
  Delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
  Delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
  Delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
  Delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_REG, reg_base_addr);
  Delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x03);
 Delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
  Delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  Delay_ms(6);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
  Delay_ms(7);
}
 uint8_t IST8310_Init(void)
{
  MPU6500_Write_Reg(USER_CTRL, 0x30);
  Delay_ms(10);
  MPU6500_Write_Reg(MPU6500_I2C_MST_CTRL, 0x0d);
  Delay_ms(10);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
  Delay_ms(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
  Delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x01);
  if(IST8310_DEVICE_ID_A != IST_Reg_Read_By_MPU(IST8310_WHO_AM_I))
    return 1; //error
  Delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFA, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFA) != 0x00)
    return 2;
  Delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFB) != 0x00)
    return 3;
  Delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_AVGCNTL, 0x24);
  if(IST_Reg_Read_By_MPU(IST8310_AVGCNTL) != 0x24)
    return 4;
	Delay_ms(10);
  
  IST_Reg_Write_By_MPU(IST8310_PDCNTL, 0xc0);
  if(IST_Reg_Read_By_MPU(IST8310_PDCNTL) != 0xc0)
    return 5;
 Delay_ms(10);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  Delay_ms(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  Delay_ms(10);
  
  MPU_Auto_Read_IST_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
  Delay_ms(100);
  return 0;
}
 
 
void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
{
	uint8_t i = 0;
	int32_t sum=0;

	for(i=1;i<10;i++)
	{
		HMC5883_FIFO[0][i-1]=HMC5883_FIFO[0][i];
		HMC5883_FIFO[1][i-1]=HMC5883_FIFO[1][i];
		HMC5883_FIFO[2][i-1]=HMC5883_FIFO[2][i];
	}
	HMC5883_FIFO[0][9]= x;//���µ����ݷ��õ� ���ݵ������
	HMC5883_FIFO[1][9]= y;
	HMC5883_FIFO[2][9]= z;
	
	for(i=0;i<10;i++)//��ǰ����ĺϣ���ȡƽ��ֵ
	{	
		 sum+=HMC5883_FIFO[0][i];
	}
	HMC5883_FIFO[0][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=HMC5883_FIFO[1][i];
	}
	HMC5883_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=HMC5883_FIFO[2][i];
	}
	HMC5883_FIFO[2][10]=sum/10;
	//����ȫ��ΪδУ׼����
	if(MagMaxMinData.MinMagX>HMC5883_FIFO[0][10])
	{
		MagMaxMinData.MinMagX=(int16_t)HMC5883_FIFO[0][10];
	}
	if(MagMaxMinData.MinMagY>HMC5883_FIFO[1][10])
	{
		MagMaxMinData.MinMagY=(int16_t)HMC5883_FIFO[1][10];
	}
	if(MagMaxMinData.MinMagZ>HMC5883_FIFO[2][10])
	{
		MagMaxMinData.MinMagZ=(int16_t)HMC5883_FIFO[2][10];
	}

	if(MagMaxMinData.MaxMagX<HMC5883_FIFO[0][10])
	{
		MagMaxMinData.MaxMagX=(int16_t)HMC5883_FIFO[0][10];		
	}
	if(MagMaxMinData.MaxMagY<HMC5883_FIFO[1][10])
	{
		MagMaxMinData.MaxMagY = HMC5883_FIFO[1][10];
	}
	if(MagMaxMinData.MaxMagZ<HMC5883_FIFO[2][10])
	{
		MagMaxMinData.MaxMagZ=(int16_t)HMC5883_FIFO[2][10];
	}		
}


 void IST8310_getRaw(int16_t *x,int16_t *y,int16_t *z)
{
	uint8_t ist_buff[6];
	ist_buff[0] = IST_Reg_Read_By_MPU(IST8310_R_XL);//Delay_ms(3);
	ist_buff[1] = IST_Reg_Read_By_MPU(IST8310_R_XM);//Delay_ms(3);
	ist_buff[2] = IST_Reg_Read_By_MPU(IST8310_R_YL);//Delay_ms(3);
	ist_buff[3] = IST_Reg_Read_By_MPU(IST8310_R_YM);//Delay_ms(3);
	ist_buff[4] = IST_Reg_Read_By_MPU(IST8310_R_YL);//Delay_ms(3);
	ist_buff[5] = IST_Reg_Read_By_MPU(IST8310_R_YM);
//	values[0]=BYTE16(s16, ist_buff[0],ist_buff[1]);
//	values[1]=BYTE16(s16, ist_buff[2],ist_buff[3]);
//	values[2]=BYTE16(s16, ist_buff[4],ist_buff[5]);
	HMC58X3_newValues(BYTE16(s16, ist_buff[0],ist_buff[1]),BYTE16(s16, ist_buff[2],ist_buff[3]),BYTE16(s16, ist_buff[4],ist_buff[5]));
	//printf("X:%f,Y:%f,Z:%f\n",values[0],values[1],values[2]);
	*x = HMC5883_FIFO[0][10];
	*y = HMC5883_FIFO[1][10]; 
	*z = HMC5883_FIFO[2][10]; 
}
void IST8310_getlastValues(int16_t *hx,int16_t *hy, int16_t *hz)
{
	  *hx = HMC5883_FIFO[0][10];
    *hy = HMC5883_FIFO[1][10]; 
    *hz = HMC5883_FIFO[2][10]; 
}
 void IST8310_mgetValues(volatile float *arry) 
{
    int16_t xr,yr,zr;
    IST8310_getRaw(&xr, &yr, &zr);
    arry[0]= HMC5883_lastx=((float)(xr - MagSavedCaliData.MagXOffset)) ;
    arry[1]= HMC5883_lasty=((float)(yr - MagSavedCaliData.MagYOffset)) ;
    arry[2]= HMC5883_lastz=((float)(zr - MagSavedCaliData.MagZOffset)) ;
}

void SPI5_Init(void)
{	 
  GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 																	//MPU6500Ƭѡ�ź�
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 		 											
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);	
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI5, ENABLE);//ʹ��SPI5ʱ��
	

	
  //GPIOFF7,8,9��ʼ������
  GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;//PF7~8���ù������	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOF, &GPIO_InitStructure);//��ʼ��
	
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource7,GPIO_AF_SPI1); 
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource8,GPIO_AF_SPI1); 
 	GPIO_PinAFConfig(GPIOF,GPIO_PinSource9,GPIO_AF_SPI1); 
 
	//����ֻ���SPI�ڳ�ʼ��
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//��λSPI5
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI5,DISABLE);//ֹͣ��λSPI5

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
	SPI_Init(SPI5, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
 
	SPI_Cmd(SPI5, ENABLE); //ʹ��SPI����

	SPI5_Read_Write_Byte(0xff);//��������		 
}
/*
 * ��������MPU6500_Write_Reg
 * ����  ��SPIд��Ĵ���
 * ����  ��reg:ָ���ļĴ�����ַ��value��д���ֵ
 * ���  ��status������״ֵ̬
 */ 
u8 MPU6500_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;
	MPU6500_CS(0);  										//ʹ��SPI����
	status = SPI5_Read_Write_Byte(reg); //����д����+�Ĵ�����
	SPI5_Read_Write_Byte(value);				//д��Ĵ���ֵ
	MPU6500_CS(1);  										//��ֹMPU6500
	return(status);											//����״ֵ̬
}


/*
 * ��������MPU6500_Read_Reg
 * ����  ��SPI��ȡ�Ĵ���
 * ����  ��reg:ָ���ļĴ�����ַ
 * ���  ��reg_val��reg�Ĵ�����ַ��Ӧ��ֵ
 */ 
u8 MPU6500_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;
	MPU6500_CS(0);  										//ʹ��SPI����
	SPI5_Read_Write_Byte(reg | 0x80); 	//���Ͷ�����+�Ĵ�����
	reg_val = SPI5_Read_Write_Byte(0xff); //��ȡ�Ĵ���ֵ
	MPU6500_CS(1);  									  //��ֹMPU6500
	return(reg_val);
}

/*
 * ��������SPI1_Read_Write_Byte
 * ����  ����дһ���ֽ�
 * ����  ��TxData:Ҫд����ֽ�
 * ���  ����ȡ�����ֽ�
 */ 
u8 SPI5_Read_Write_Byte(uint8_t TxData)
{		
	u8 retry = 0;				 	
	while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET) 	//���ָ����SPI��־λ�������:���ͻ���ձ�־λ
		{
		retry++;
		if(retry > 250)	return 0;
		}			  
	SPI_I2S_SendData(SPI5, TxData); 																//ͨ������SPIx����һ������
	retry = 0;

	while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_RXNE) == RESET) //���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
	{
		retry++;
		if(retry > 250) return 0;
	}	  						    
	return SPI_I2S_ReceiveData(SPI5); 															//����ͨ��SPIx������յ�����					    
}

