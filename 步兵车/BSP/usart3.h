#ifndef __USART3_H__
#define __USART3_H__
#include "main.h"
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )


void USART3_Configuration(void);
void Usart3_Send(unsigned char *DataToSend ,u8 data_num);
void dataTransfer(u8 _temp);
extern u8 info[];
extern int  shibieMode,shibieMode2;
extern float shibiePitch;
extern float shibieYaw;
void dataAnl(u8 *com);
#endif
