#include "servo.h"
//#include "usart.h"
#include "delay.h"
//#include "common.h"

#define MAXPWM 2505		 //������PWM��������2.5ms�궨��
#define MOTOR_NUM 9

extern u8 date[3];

u8 count1=0,count2,count3;				 //ÿ����������8·����Ⱥ�ֵ����
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;	 //????? TIM??? ????????????
TIM_TimeBaseInitTypeDef TIM_Time1BaseStructure;

extern u16 CPWM[MOTOR_NUM];
extern u8 flag_vpwm;				//��ʾ�����˸ø���pwm[]��ʱ��

void Servor_GPIO_Config(void)	
{
	GPIO_InitTypeDef GPIO_InitStructure;	
 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable , ENABLE);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0|GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
}
/************************GPIO��ƽ��ת����1*****************************************/ 

void Flip_GPIO_One(void)
{
	switch(count1)								 //��20ms�Ķ���������ڷֳ�8�ݣ�ÿ2.5ms����һ�������ת
	{  											      //ÿ����ʱ������8·�����ת��3����ʱ������24·�����ת
		case 1: 
			 TIM2->ARR=CPWM[1];				 //����һ�����������ֵ��ֵ����ʱ��2			
    	 GPIO_SetBits(GPIOB,GPIO_Pin_6);  //ͬʱ���߿��ƶ��1�����ŵĵ�
  	   flag_vpwm=1;	
				break;
		
		case 2:
   	    TIM2->ARR=MAXPWM-CPWM[1]; 		 //��2.5ms��ȥPWM����ֵ�����ݸ�ֵ��ʱ��2
				GPIO_ResetBits(GPIOB,GPIO_Pin_6);//ͬʱ���Ϳ��ƶ��1���ŵĵ�ƽ 
        flag_vpwm=1;	
				break;							 //���ƶ��1��������ʣ��20ms-CPM[0]ʱ���ڽ�һֱ���ֵ͵�ƽ�����1����CPWMֵת��

		case 3:	
			TIM2->ARR=CPWM[2]; 
				GPIO_SetBits(GPIOB,GPIO_Pin_7);
        flag_vpwm=1;			
				break;
		
		case 4:	
			TIM2->ARR=MAXPWM-CPWM[2];  
				GPIO_ResetBits(GPIOB,GPIO_Pin_7); 
		    flag_vpwm=1;	
				break;

		case 5:	
			TIM2->ARR=CPWM[3];  
				GPIO_SetBits(GPIOB,GPIO_Pin_8); 
				flag_vpwm=1;	
				break;
		
		case 6:	
			TIM2->ARR=MAXPWM-CPWM[3];  
				GPIO_ResetBits(GPIOB,GPIO_Pin_8);
				flag_vpwm=1;	
				break;

		case 7:	
			 TIM2->ARR=CPWM[4];  
			  GPIO_SetBits(GPIOB,GPIO_Pin_9); 
				flag_vpwm=1;	
				break;
		
		case 8:	
			 TIM2->ARR=MAXPWM-CPWM[4];  
				GPIO_ResetBits(GPIOB,GPIO_Pin_9);
				flag_vpwm=1;	
				break;

		case 9:	
			TIM2->ARR=CPWM[5];  
				GPIO_SetBits(GPIOA,GPIO_Pin_0); 
				flag_vpwm=1;	
				break;
		
		case 10:
			TIM2->ARR=MAXPWM-CPWM[5];  
				GPIO_ResetBits(GPIOA,GPIO_Pin_0);
				flag_vpwm=1;	
				break;

		case 11:
			TIM2->ARR=CPWM[6];  
				GPIO_SetBits(GPIOA,GPIO_Pin_1); 
				flag_vpwm=1;	
				break;
		
		case 12:
			TIM2->ARR=MAXPWM-CPWM[6];  
				GPIO_ResetBits(GPIOA,GPIO_Pin_1);
				flag_vpwm=1;	
				break;

		case 13:
			TIM2->ARR=CPWM[7];  

				flag_vpwm=1;	
				break;
		
		case 14:
			  TIM2->ARR=MAXPWM-CPWM[7];  

				flag_vpwm=1;	
				break;

		case 15:
			TIM2->ARR=CPWM[8];  

				flag_vpwm=1;	
				break;
		
		case 16:
			TIM2->ARR=MAXPWM-CPWM[8]; 

				flag_vpwm=1;	
				count1=0; 
				break;
		default:break;
	}	
	//count1++;
}


#if 0
/**********************************************************************************/
/************************GPIO��ƽ��ת����2*****************************************/ 
void Flip_GPIO_Two(void)
{
	switch(count2)
	{  		
		case 1: TIM3->ARR=CPWM[8];				 //����ͬ��
				GPIO_SetBits(GPIOA,GPIO_Pin_8);  
				break;
		
		case 2:	TIM3->ARR=MAXPWM-CPWM[8]; 
				GPIO_ResetBits(GPIOA,GPIO_Pin_8); 
				break;

		case 3:	TIM3->ARR=CPWM[9]; 
				GPIO_SetBits(GPIOA,GPIO_Pin_9); 
				break;
		
		case 4:	TIM3->ARR=MAXPWM-CPWM[9];  
				GPIO_ResetBits(GPIOA,GPIO_Pin_9); 
				break;

		case 5:	TIM3->ARR=CPWM[10];  
				GPIO_SetBits(GPIOB,GPIO_Pin_9); 
				break;
		
		case 6:	TIM3->ARR=MAXPWM-CPWM[10];  
				GPIO_ResetBits(GPIOB,GPIO_Pin_9);
				break;

		case 7:	TIM3->ARR=CPWM[11];  
				GPIO_SetBits(GPIOB,GPIO_Pin_8); 
				break;
		
		case 8:	TIM3->ARR=MAXPWM-CPWM[11];  
				GPIO_ResetBits(GPIOB,GPIO_Pin_8);
				break;

		case 9:	TIM3->ARR=CPWM[12];  
				GPIO_SetBits(GPIOB,GPIO_Pin_7); 
				break;
		
		case 10:TIM3->ARR=MAXPWM-CPWM[12];  
				GPIO_ResetBits(GPIOB,GPIO_Pin_7);
				break;

		case 11:TIM3->ARR=CPWM[13];  
				GPIO_SetBits(GPIOB,GPIO_Pin_6); 
				break;
		
		case 12:TIM3->ARR=MAXPWM-CPWM[13];  
				GPIO_ResetBits(GPIOB,GPIO_Pin_6);
				break;

		case 13:TIM3->ARR=CPWM[14];  
				GPIO_SetBits(GPIOB,GPIO_Pin_5); 
				break;
		
		case 14:TIM3->ARR=MAXPWM-CPWM[14];  
				GPIO_ResetBits(GPIOB,GPIO_Pin_5);
				break;

		case 15:TIM3->ARR=CPWM[15];  
				GPIO_SetBits(GPIOB,GPIO_Pin_4); 
				break;
		
		case 16:TIM3->ARR=MAXPWM-CPWM[15]; 
				GPIO_ResetBits(GPIOB,GPIO_Pin_4); 
				count2=0; 
				break;
	}	
}
#endif

/**********************************************************************************/   
/************************������ƺ���1*********************************************/
void Servo1(void)
{		
	count1++; 
	Flip_GPIO_One();						 //��תIO��ƽ

}
/**********************************************************************************/

#if 0
/************************������ƺ���2*********************************************/
void Servo2(void)
{		
	count2++;	
	Flip_GPIO_Two();
}
/**********************************************************************************/
#endif

