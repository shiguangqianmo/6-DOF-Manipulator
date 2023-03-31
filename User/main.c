#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "stm32f10x.h"	
#include "led.h"
#include "delay.h"
#include "servo.h"
#include "tim.h"
#include "ultrasonic.h"
#include "usart.h"

#define pi 3.1415926
#define RIGHT 1
#define FALSE 0

/**************ָ������*********************/
#define MOVE  1				//�ƶ�
#define MEASURE 2			//ȥץȡ����ࣩ
#define CATCH 3				//צ�Ӻ���
#define LOOSEN 4			//צ���ɿ�
#define CATCH_UP 5		//ץ����
#define CENTER 6			//����
//**********************************************

//�����������ر���
extern u8  TIM3CH2_CAPTURE_STA;		//���벶��״̬		    				
extern u16	TIM3CH2_CAPTURE_VAL;	//���벶��ֵ	

//���ڽ����ж���ر���
u8 Data_Flag=0;		//���ݰ����ձ�־λ  
u16 Num=0;				//���ݸ�����¼λ
u16 Length;				//����
u8 Data;					//���ݻ���
u8 Cheak=0;				//У��
u8 Data_Buffer[9];			//ָ������
u8 Receive_Finish_Flag=0;	//������ɱ�־λ��1Ϊ��ɣ�0Ϊδ���

//��е����ر���
double l1 = 10.4, l2 = 9.65;
u8 date[3];
u16 Center_PWM[9] = {1500,1500,1500,1500,1500,1500,1500,1500,1500};		//����λ�õ�PWM
u16 Target_PWM[9] = {1500,1500,1500,1500,1500,1500,1500,1500,1500};		//Ŀ��λ�õ�PWM
u16 CPWM[9] = {1500,1500,1500,1500,1500,1500,1500,1500,1500};					//��ǰλ�õ�PWM
u8 flag_vpwm;				//��ʾ�����˸ø���pwm[]��ʱ��
double target[3] = {-1, -8, 10};			//Ŀ��λ��
double theta[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};	//�ؽڽǶȣ���һ��û�ã�-135~+135
double cul_theta[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};	//��ǰ�ؽڽǶ�

//��е���˶���غ���
void Cal_Tar_Theta(void);		//���㵽��Ŀ����Ӧ�ĸ����ؽڽǵĴ�С
void PID_Col_Theta(void);		//PID���ƹؽڴ��
void Angle_To_Pwm(void);		//���ؽڽǵ�ֵת��ΪĿ��PWM��ֵ
void To_Center(void);				//����PWMתΪ����PWM
void To_Target(void);				//��ǰPWMתΪĿ��PWM

double Distance_Measure(void);		//��������ຯ��

int main(void)
{
	u8 Receive_Right_Flag=FALSE;//������ȷ��־λ��FALSEΪ0��RIGHTΪ1��
	u8 i; 											//�м���ʱ����
	double distence;					//����
	u8 instruction=0;						//ָ�MOVE(0x01) / CATCH(0x02)
	u8 measure_count = 0;		//������
	double Dis[10];
	u8 count = 0;					//��Ч������
	
	/*****************Ӳ����ʼ��&����Ƕȹ���**********************/
	delay_init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);
	LED_Init();
	Servor_GPIO_Config();
	Timer_Init();
	Triger_PB3_Init();
	TIM3_Cap_Init(0XFFFF,72-1);
	
	/********************��ʼ***************************/
	delay_ms(500);
	//Cal_Tar_Theta();
	printf("start!\r\n");
	while(1)
	{
		//delay_ms(10);
		Led_Test();

		PID_Col_Theta();
		Angle_To_Pwm();
		To_Target();

		//Receive_Right_Flag = 1;
/***********************��⴮�ڽ���ָ��***************************/		
		if(Receive_Right_Flag == FALSE)//��������δУ��ͨ����������������У��
		{
			if(Receive_Finish_Flag == FINISHED)//������ɲŽ���У��
			{
				for(i=1;i<=Real_Length-2;i++)//��ͣ�����У��
				{
					Cheak=Cheak+Data_Buffer[i];			
				}
				if((Cheak == Data_Buffer[Real_Length-1])&&(Length==Real_Length))//У����ȷ
				{				
					Receive_Right_Flag=RIGHT;						//���ڽ��ճɹ�һ����Чָ��˳�whileѭ����ֱ��ִ������Ӧָ����׼��������һ��
					printf("check!\r\n");								//��ӡУ��ɹ�
					Receive_Finish_Flag = UNFINISHED;		//׼����һ�ν���
					//Receive_Right_Flag = FALSE;			//׼����һ�ν���ѭ��
					
					/**************��ȡָ��****************/
					instruction = Data_Buffer[1];
					/**************************************/
					
					Cheak = 0;
					Length = 0;
				}
				else//У�鲻��ȷ
				{
					/*�����������ź�*/
					/*�������ݳ�ʼ��*/
					Receive_Finish_Flag=FALSE;
					Cheak=0;//У��λ����	
					Length=0;//����λ����
				}
			}
		}	

/******************************ִ�д���ָ��*******************************/
		switch(instruction)
		{
			case MOVE:			//01
			{
				/*�˶���ָ��λ��*/
				printf("move to the target!\r\n");	
				
				// ��ȡָ���е�Ŀ��λ��
				//xֻ��Ϊ����
				target[0] = -((double)(Data_Buffer[2]&(0x7f)))/10;
				//y��zֻ��Ϊ����
				target[1] = ((double)(Data_Buffer[3]&(0x7f)))/10;
				target[2] = ((double)(Data_Buffer[4]&(0x7f)))/10;
				
				Cal_Tar_Theta();		//����ؽڽ�
				
				instruction = 0;							//���ָ��
				Receive_Right_Flag = FALSE;		//���������һ��ָ��
				break;
			}
			
			case MEASURE:			//02
			{
				while(measure_count < 10)		//û��⹻10��
				{
					Ultrasonic_Trig();	//����������
					delay_ms(500);
					if(TIM3CH2_CAPTURE_STA&0X80)//�ɹ�������һ��������
					{
						distence = Distance_Measure();
						Dis[measure_count] = distence;
						printf("distence:%f cm\r\n",distence);//��ӡ�ܵĸߵ�ƽʱ��		
					}
					measure_count++;
				}
					/*ץȡ����*/
					printf("catch the goal!\r\n");
					distence = 0;
					for(i=0; i<10;i++)
					{
						if(Dis[i] <30 && Dis[i]>10)
						{
							count++;
							distence = distence + Dis[i];
						}					
					}
					for(i=0; i<10;i++)
					{
						Dis[i] = 0;
					}
					distence = distence / count;	//����ƽ������
					count = 0;
					printf("mean distence:%f cm\r\n",distence);//��ӡƽ������
					
					/************������������*************/
					//distence = distence - 4.8;
					distence = distence - 5;
					if (distence>=6 && distence <=20)
					{
						target[0] = -distence;				//�趨x������
						target[1] = target[2] = 0;		//y��z�������Ϊ0
						Cal_Tar_Theta();
					}
					else
						printf("The object is in an unreasonable position!\r\n");
			
										
					measure_count = 0;
					instruction = 0;							//���ָ��
					Receive_Right_Flag = FALSE;		//���������һ��ָ��
				
				break;
			}
			
			case CATCH:				//03
			{
				printf("catch!\r\n");
				theta[6] = 100;					//צ�Ӻ���
				
				instruction = 0;							//���ָ��
				Receive_Right_Flag = FALSE;		//���������һ��ָ��			
				break;
			}
			case LOOSEN:					//04
			{
				printf("loosen!\r\n");
				theta[6] = 0;						//צ���ɿ�
				
				instruction = 0;							//���ָ��
				Receive_Right_Flag = FALSE;		//���������һ��ָ��			
				break;
			}
			
			case CATCH_UP:			//05
			{
				printf("catch up!\r\n");
				for (i=1;i<=4;i++)
				{
					theta[i] = 0;
				}
				
				instruction = 0;							//���ָ��
				Receive_Right_Flag = FALSE;		//���������һ��ָ��			
				break;
			}
			
			case CENTER:					//06
			{
				/*����*/
				printf("go to the center!\r\n");
				for (i=1;i<=6;i++)
				{
					theta[i] = 0;
				}
				instruction = 0;							//���ָ��
				Receive_Right_Flag = FALSE;		//���������һ��ָ��
				break;
			}
			default:break;
		}
		
	}	
}


double Distance_Measure(void)
{
	u32 temp;
	double distence;
	temp=TIM3CH2_CAPTURE_STA&0X3F;
	temp*=65536;//���ʱ���ܺ�
	temp+=TIM3CH2_CAPTURE_VAL;//�õ��ܵĸߵ�ƽʱ��
	distence = (double)temp * 0.0001 * 340 / 2;
	TIM3CH2_CAPTURE_STA=0;//������һ�β���
	return distence;
}

//���㵽��Ŀ����Ӧ�ĸ����ؽڽǵĴ�С
void Cal_Tar_Theta(void)
{
	int i;
	double x0,y0,beta,fai;
	theta[1] = atan(target[1] / target[0]);			//����theta1
	
	//����theta3
	x0 = pow((pow(target[0],2)+pow(target[1],2)),0.5);
	y0 = target[2];
	theta[3] = acos((pow(x0,2)+pow(y0,2)-pow(l1,2)-pow(l2,2))/(2*l1*l2));
	
	//����theta2
	beta = atan(y0/x0);
	fai = acos((pow(x0,2)+pow(y0,2)+pow(l1,2)-pow(l2,2))/(2*l2*pow((pow(x0,2)+pow(y0,2)),0.5)));
	theta[2] = pi/2 - (beta + fai);
	
	//����theta4
	//theta[4] = pi - theta[2] - theta[3];
	theta[4] = theta[3] + theta[2] - pi/2;
	
	for (i = 1; i <= 4; i++)
	{
		theta[i] = theta[i] / pi * 180;
	}
}

//PID���ƹؽڴ��
void PID_Col_Theta(void)
{
	double Kp = 0.04;
	int i;
	for (i = 0; i < 9; i++)
	{
		cul_theta[i] = cul_theta[i] + Kp*(theta[i] - cul_theta[i]);		
	}
}


//���ؽڽ�(�Ƕ�)��ֵת��ΪĿ��PWM��ֵ
void Angle_To_Pwm(void)
{
	int i;
	for(i = 0; i < 9; i++)
	{
		if(i == 3)
			Target_PWM[i] = 1500 - (int)(cul_theta[i] * 7.41);		//ȷ��������
		else
			Target_PWM[i] = 1500 + (int)(cul_theta[i] * 7.41);
	}
}

//����PWMתΪ����PWM
void To_Center(void)
{
	int i;
	for (i = 0; i < 9; i++)
	{
		CPWM[i] = Center_PWM[i];
	}
}

//��ǰPWMתΪĿ��PWM
void To_Target(void)				
{
	int i;
	for (i = 0; i < 9; i++)
	{
		CPWM[i] = Target_PWM[i];
	}
}



