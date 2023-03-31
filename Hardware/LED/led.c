#include "led.h"
#include "delay.h"


/**************************************************************************
�������ܣ�LED IO��ʼ��
��ڲ�������
����  ֵ���� 
**************************************************************************/
void LED_Init(void)
{
 
	GPIO_InitTypeDef GPIO_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//ʹ��PB�˿�ʱ��
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_14;     //LED-->PB14
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //����Ϊ�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //IO���ٶ�Ϊ50MHZ
 	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��PB14	
	GPIO_SetBits(GPIOB,GPIO_Pin_14);				//PB14 �����

}
 /**************************************************************************
�������ܣ�LED ���Ժ��� ����LED 500msȻ��Ϩ��500ms
��ڲ�������
����  ֵ���� 
**************************************************************************/
void Led_Test(void)
{
		LED1(ON);  //GPIO_ResetBits(GPIOB,GPIO_Pin_14)  LED =0;
	  delay_ms(70);
	  LED1(OFF); //GPIO_SetBits(GPIOB,GPIO_Pin_14);   LED =1;
	  delay_ms(70);
	                              
	 
}
