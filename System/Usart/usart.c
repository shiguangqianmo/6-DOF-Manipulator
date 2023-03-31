#include "stm32f10x.h" 
#include "usart.h"	
#include "delay.h"

#define PACKET_HEADER 1
#define DATA_RECEIVE 2




extern u8 Data_Flag;//数据包接收标志位  
extern u16 Num;//数据个数记录位
extern u16 Length;//长度
extern u8 Data;//数据缓存
extern u8 Data_Buffer[9];//指令数组
extern u8 Receive_Finish_Flag;//接收完成标志位





//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

void uart_init(u32 bound){
  //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART1_TX   GPIOA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
    //USART1_RX	  GPIOA.10初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

    //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
    //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

    USART_Init(USART1, &USART_InitStructure); //初始化串口1
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
    USART_Cmd(USART1, ENABLE);                    //使能串口1 
	
/*********************************************************************************************************/	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART2，GPIOA时钟
//  
//	//USART1_TX   GPIOA.2
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.9
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
//	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
//   
//    //USART1_RX	  GPIOA.3初始化
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA10
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
//    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

//    //Usart1 NVIC 配置
//    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
//  
//    //USART 初始化设置

//	USART_InitStructure.USART_BaudRate = bound;//串口波特率
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
//	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

//    USART_Init(USART2, &USART_InitStructure); //初始化串口1
//    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
//    USART_Cmd(USART2, ENABLE);                    //使能串口1 

}


//void USART1_IRQHandler(void)
//{
//	u8 data;
//	if(USART_GetITStatus(USART1,USART_IT_RXNE) != 0)
//	{
//		data=USART_ReceiveData(USART1);//缓存数据
////		if (data==0xff)
////		{
//			USART_SendData(USART1,data);
//			while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==0);
////		}
//	}
//}
//void USART2_IRQHandler(void)
//{
//	if(USART_GetITStatus(USART2,USART_IT_RXNE) != 0)
//	{
//		Data=USART_ReceiveData(USART2);//缓存数据
//		USART_SendData(USART2,Data);
//		while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==0);
//	}
//}

void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	static u8 state=PACKET_HEADER;
	if(USART_GetITStatus(USART1,USART_IT_RXNE) != 0)
	{
		Data=USART_ReceiveData(USART1);//缓存数据
/**********************包头识别************************/
		if(state==PACKET_HEADER)
		{
			if(Data==0xff)
			{
				USART_SendData(USART1,Data);
				while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==0);
				
				Data_Flag++;
			}
			else
			{
				if(Data_Flag != 2)
				{
					Data_Flag=0;
				}
				else
				{
					state=DATA_RECEIVE;
				}
			}
		}
/***********************数据接收*************************/
		if(state==DATA_RECEIVE)
		{
			if(Data==0x00)//接收到停止位
			{
				USART_SendData(USART1,Data);
				while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==0);
				
				Real_Length=Num;
				
				Num=0;
				Data_Flag=0;
				state=PACKET_HEADER;
				Receive_Finish_Flag=FINISHED;
			}
			else//未接收到停止位
			{
				USART_SendData(USART1,Data);
				while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==0);
				
				Num++;
				if(Num==1)
				{
					Length=Data;
				}
				else
				{
					Data_Buffer[Num-1]=Data;
				}
			}
		}
	}
} 

//void USART1_IRQHandler(void)
//{
//	
//}


