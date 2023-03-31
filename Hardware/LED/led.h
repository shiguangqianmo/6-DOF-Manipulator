#ifndef __LED_H
#define __LED_H	 

#include "stm32f10x.h"
#include "sys.h"

#define ON  0
#define OFF 1

//#define	LED    PBout(14)  //对PB13口输出操作		
#define LED1(a) if(a) \
											GPIO_SetBits(GPIOB,GPIO_Pin_14); \
								else \
											GPIO_ResetBits(GPIOB,GPIO_Pin_14)

void LED_Init(void);//初始化
void Led_Test(void);
		 				    
#endif
