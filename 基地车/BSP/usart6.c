#include "main.h"
/*----USART6----PG9-----TX */
/*----USART6----PG14-----RX */
/*----BaudRate------ 115200*/

void Usart6_Init(u32 br_num)
{ 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
	NVIC_Init(&NVIC_InitStructure);	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
 
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6); 
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6); 
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOG,&GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = br_num;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART6, &USART_InitStructure); 
	
  USART_Cmd(USART6, ENABLE);
	
	
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);



}

u8 Rx6_Buf[256];
u8 Tx6_Buf[256];
u8 count6=0;
u8 TxCounter6=0;
void USART6_IRQHandler(void)
{
	u8 com_data;

	if(USART6->SR & USART_SR_ORE)
	{
		com_data = USART6->DR;
	}
	if( USART_GetITStatus(USART6,USART_IT_RXNE) )
	{
		
		USART_ClearITPendingBit(USART6,USART_IT_RXNE);

		com_data = USART6->DR;
		
	//		Set_Gimbal_Current(CAN1,-50*com_data,-50*com_data);
	//	PWM1 =1000+ com_data;PWM2 =1000+com_data ;
	//		Set_CM_Speed(CAN1,100*com_data,100*com_data,100*com_data,100*com_data);
		Usart2_Send_Byte(com_data);
		
	}
	if( USART_GetITStatus(USART6,USART_IT_TC ) )
	{
		USART6->DR = Tx6_Buf[TxCounter6++];     
		if(TxCounter6 == count6)
		{
			USART6->CR1 &= ~USART_CR1_TCIE;		
		}
	}
}
void Usart6_Send(unsigned char *DataToSend ,u8 data_num)
{
  u8 i;
	for(i=0;i<data_num;i++)
	{
		Tx6_Buf[count6++] = *(DataToSend+i);
	}

	if(!(USART6->CR1 & USART_CR1_TXEIE))
	{
		USART_ITConfig(USART6, USART_IT_TC, ENABLE); 
	}

}

void Usart6_Send_Byte(unsigned char DataToSend)
{
	Tx6_Buf[count6++] = DataToSend;
	if(!(USART6->CR1 & USART_CR1_TXEIE))
	{
		USART_ITConfig(USART6, USART_IT_TC, ENABLE); 
	}
}


