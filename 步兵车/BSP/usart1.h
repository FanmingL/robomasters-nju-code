#ifndef __USART1_H__
#define __USART1_H__
#include "main.h"
#define PITCH_MAX 19.0f
#define YAW_MAX 720.0f//720.0				//cyq:ÔÆÌ¨½Ç¶ÈµÄ·¶Î§
/*
*********************************************************************************************************
*                                               MACROS
*********************************************************************************************************
*/

#define  BSP_USART1_DMA_RX_BUF_LEN               30u                   
 
#define BSP_USART1_RX_BUF_SIZE_IN_FRAMES         (BSP_USART1_RX_BUF_SIZE / RC_FRAME_LENGTH)
#define  RC_FRAME_LENGTH                            18u

#define KEY_W 0x01
#define KEY_S 0x02
#define KEY_A 0x04
#define KEY_D 0x08
#define KEY_SHIFT 0x10
#define KEY_CTRL 0x20
#define KEY_Q 0x40
#define KEY_E 0x80
#define KEY_R 0x100
#define KEY_F 0x200
#define KEY_G 0x400
#define KEY_Z 0x800
#define KEY_X 0x1000
#define KEY_C 0x2000
#define KEY_V 0x4000
#define KEY_B 0x8000
void printKey(void);
/*
*********************************************************************************************************
*                                             FUNCTION PROTOTYPES
*********************************************************************************************************
*/
void USART1_IRQHandler(void);
static void USART1_FIFO_Init(void);
void *USART1_GetRxBuf(void);
void USART1_Configuration(uint32_t baud_rate);
void RemoteDataPrcess(uint8_t *pData);

#endif
