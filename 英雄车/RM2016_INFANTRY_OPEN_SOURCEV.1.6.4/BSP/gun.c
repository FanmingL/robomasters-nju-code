#include "stm32f4xx.h"
#include "gun.h"

/*-LEFT---(PH6---TIM12_CH1)--*/
/*-RIGHT--(PH9--TIM12_CH2)--*/

/*-----PH11(TIM5 CH2)----PH12(TIM5 CH3)------*/

void PWM_Configuration(void)
{
    GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH ,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);   //90MHz

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOH,&gpio);

    GPIO_PinAFConfig(GPIOH,GPIO_PinSource6,GPIO_AF_TIM12);
    GPIO_PinAFConfig(GPIOH,GPIO_PinSource9,GPIO_AF_TIM12); 
	
    tim.TIM_Prescaler = 84-1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 2500-1;   //2.5ms
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM12,&tim);
		
    oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputState_Disable;
    oc.TIM_Pulse = 1000-1;
    oc.TIM_OCPolarity = TIM_OCPolarity_Low;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Set;
    TIM_OC1Init(TIM12,&oc);
    TIM_OC2Init(TIM12,&oc);
    
    TIM_OC1PreloadConfig(TIM12,TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM12,TIM_OCPreload_Enable);
           
    TIM_ARRPreloadConfig(TIM12,ENABLE);
		
    TIM_Cmd(TIM12,ENABLE);
		////////////////////
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH ,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);   //90MHz

    gpio.GPIO_Pin =GPIO_Pin_10 |GPIO_Pin_11 | GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOH,&gpio);

    GPIO_PinAFConfig(GPIOH,GPIO_PinSource10,GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOH,GPIO_PinSource11,GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOH,GPIO_PinSource12,GPIO_AF_TIM5); 
	
    tim.TIM_Prescaler = 84-1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 2500-1;   //2.5ms
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM5,&tim);
		
    oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputState_Disable;
    oc.TIM_Pulse = 1000-1;
    oc.TIM_OCPolarity = TIM_OCPolarity_Low;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Set;
    TIM_OC2Init(TIM5,&oc);
    TIM_OC3Init(TIM5,&oc);
    TIM_OC1Init(TIM5,&oc);
    TIM_OC1PreloadConfig(TIM5,TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM5,TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM5,TIM_OCPreload_Enable);
           
    TIM_ARRPreloadConfig(TIM5,ENABLE);
		
    TIM_Cmd(TIM5,ENABLE);
		InitFrictionWheel();
		DClose();
}
void duo(void){

	static int _step=0;
	if (_step == 0){
		DUOJI = 0x58*10;
		_step++;
	}else if(_step==1){
		DUOJI = 0xa0*10;
		_step++;
	}else if(_step == 2){
		DUOJI = 0xf1*10;
		_step++;
	}else if (_step ==3){
		DUOJI = 0xa0*10;
		_step=0;
	}


}
