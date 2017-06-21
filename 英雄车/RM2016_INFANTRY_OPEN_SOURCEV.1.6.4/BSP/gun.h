#ifndef __GUN_H__
#define __GUN_H__

#ifndef FRICTION_WHEEL
#define FRICTION_WHEEL
#endif 

#if defined(FRICTION_WHEEL)

#define PWM1  TIM12->CCR1
#define PWM2  TIM12->CCR2
#define PWM3  TIM5->CCR2
#define PWM4  TIM5->CCR3
#define DUOJI  TIM5->CCR1

#define InitFrictionWheel()     \
        PWM1 = 1000;             \
        PWM2 = 1000;             \
				PWM3 = 1000;             \
				PWM4 = 1000;
#define SetFrictionWheelSpeed(x,y) \
        PWM1 = x;                \
        PWM2 = x;								\
				PWM3 = y;								\
				PWM4 = y;								
#define DUOJIINIT()   (DUOJI =  (0xbf*10))
#define DOpen()				(DUOJI =  (0xa0*10))
#define DClose()			(DUOJI = (0xbf*10))

#endif 

void PWM_Configuration(void);
 void duo(void);
#endif /* __GUN_H__*/

