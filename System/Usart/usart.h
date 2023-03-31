#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f10x.h" 

#define FINISHED 1
#define UNFINISHED 0

#define Real_Length  Data_Buffer[0] 

void uart_init(u32 bound);
#endif

