#include "main.h"

void BSP_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);     
	PWM_Configuration();
	Led_Configuration();            
	Laser_Configuration();
	TIM2_Configuration();		
	delay_ms(100); 
	while(!MPU6500_Init());
	delay_ms(100);   
	while(IST8310_Init());	
	MPU6500_Date_Offset(500);
	TIM6_Configuration();		
	Quad_Encoder_Configuration();		
	CAN1_Configuration();           
	CAN2_Configuration();            
	USART1_Configuration(100000);  
 Usart2_Init(115200);	
	USART3_Configuration();         
	TIM6_Start();   	
	//MPU6050_IntConfiguration();     
	//MPU6050_EnableInt();   
//	MPU6500_Date_Offset(100);
	MPU6500_Write_Reg(PWR_MGMT_2,0X00);
	
	Encoder_Start();
}

