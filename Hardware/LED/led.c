#include "led.h"
#include "delay.h"


/**************************************************************************
函数功能：LED IO初始化
入口参数：无
返回  值：无 
**************************************************************************/
void LED_Init(void)
{
 
	GPIO_InitTypeDef GPIO_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//使能PB端口时钟
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_14;     //LED-->PB14
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //设置为推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //IO口速度为50MHZ
 	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PB14	
	GPIO_SetBits(GPIOB,GPIO_Pin_14);				//PB14 输出高

}
 /**************************************************************************
函数功能：LED 测试函数 点亮LED 500ms然后熄灭500ms
入口参数：无
返回  值：无 
**************************************************************************/
void Led_Test(void)
{
		LED1(ON);  //GPIO_ResetBits(GPIOB,GPIO_Pin_14)  LED =0;
	  delay_ms(70);
	  LED1(OFF); //GPIO_SetBits(GPIOB,GPIO_Pin_14);   LED =1;
	  delay_ms(70);
	                              
	 
}
