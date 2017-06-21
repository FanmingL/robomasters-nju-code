#ifndef __GUN_H__
#define __GUN_H__

#ifndef FRICTION_WHEEL
#define FRICTION_WHEEL
#endif 

#if defined(FRICTION_WHEEL)

#define PWM1  TIM12->CCR1
#define PWM2  TIM12->CCR2

#define InitFrictionWheel()     \
        PWM1 = 1000;             \
        PWM2 = 1000;
#define SetFrictionWheelSpeed(x) \
        PWM1 = x;                \
        PWM2 = x;

#endif 


void PWM_Configuration(void);
 
#endif /* __GUN_H__*/

