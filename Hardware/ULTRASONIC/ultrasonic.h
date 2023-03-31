#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "stm32f10x.h"
#include "sys.h"

void Triger_PB3_Init(void);
void Ultrasonic_Trig(void);
void TIM3_Cap_Init(u16 arr,u16 psc);

		 				    
#endif
