#include "stm32f10x.h" 
#include "usart.h"	
#include "delay.h"

#define PACKET_HEADER 1
#define DATA_RECEIVE 2




extern u8 Data_Flag;//���ݰ����ձ�־λ  
extern u16 Num;//���ݸ�����¼λ
extern u16 Length;//����
extern u8 Data;//���ݻ���
extern u8 Data_Buffer[9];//ָ������
extern u8 Receive_Finish_Flag;//������ɱ�־λ





//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

void uart_init(u32 bound){
  //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
  
	//USART1_TX   GPIOA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
    //USART1_RX	  GPIOA.10��ʼ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

    //Usart1 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
    //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

    USART_Init(USART1, &USART_InitStructure); //��ʼ������1
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
    USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1 
	
/*********************************************************************************************************/	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART2��GPIOAʱ��
//  
//	//USART1_TX   GPIOA.2
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.9
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
//	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
//   
//    //USART1_RX	  GPIOA.3��ʼ��
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA10
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
//    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

//    //Usart1 NVIC ����
//    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
//  
//    //USART ��ʼ������

//	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
//	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

//    USART_Init(USART2, &USART_InitStructure); //��ʼ������1
//    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
//    USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���1 

}


//void USART1_IRQHandler(void)
//{
//	u8 data;
//	if(USART_GetITStatus(USART1,USART_IT_RXNE) != 0)
//	{
//		data=USART_ReceiveData(USART1);//��������
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
//		Data=USART_ReceiveData(USART2);//��������
//		USART_SendData(USART2,Data);
//		while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==0);
//	}
//}

void USART1_IRQHandler(void)                	//����1�жϷ������
{
	static u8 state=PACKET_HEADER;
	if(USART_GetITStatus(USART1,USART_IT_RXNE) != 0)
	{
		Data=USART_ReceiveData(USART1);//��������
/**********************��ͷʶ��************************/
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
/***********************���ݽ���*************************/
		if(state==DATA_RECEIVE)
		{
			if(Data==0x00)//���յ�ֹͣλ
			{
				USART_SendData(USART1,Data);
				while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==0);
				
				Real_Length=Num;
				
				Num=0;
				Data_Flag=0;
				state=PACKET_HEADER;
				Receive_Finish_Flag=FINISHED;
			}
			else//δ���յ�ֹͣλ
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


