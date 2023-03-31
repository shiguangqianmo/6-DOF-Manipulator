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

/**************指令类型*********************/
#define MOVE  1				//移动
#define MEASURE 2			//去抓取（测距）
#define CATCH 3				//爪子合上
#define LOOSEN 4			//爪子松开
#define CATCH_UP 5		//抓起来
#define CENTER 6			//归中
//**********************************************

//超声波测距相关变量
extern u8  TIM3CH2_CAPTURE_STA;		//输入捕获状态		    				
extern u16	TIM3CH2_CAPTURE_VAL;	//输入捕获值	

//串口接收中断相关变量
u8 Data_Flag=0;		//数据包接收标志位  
u16 Num=0;				//数据个数记录位
u16 Length;				//长度
u8 Data;					//数据缓存
u8 Cheak=0;				//校验
u8 Data_Buffer[9];			//指令数组
u8 Receive_Finish_Flag=0;	//接收完成标志位，1为完成，0为未完成

//机械臂相关变量
double l1 = 10.4, l2 = 9.65;
u8 date[3];
u16 Center_PWM[9] = {1500,1500,1500,1500,1500,1500,1500,1500,1500};		//归中位置的PWM
u16 Target_PWM[9] = {1500,1500,1500,1500,1500,1500,1500,1500,1500};		//目标位置的PWM
u16 CPWM[9] = {1500,1500,1500,1500,1500,1500,1500,1500,1500};					//当前位置的PWM
u8 flag_vpwm;				//表示到达了该更新pwm[]的时间
double target[3] = {-1, -8, 10};			//目标位置
double theta[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};	//关节角度，第一个没用，-135~+135
double cul_theta[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};	//当前关节角度

//机械臂运动相关函数
void Cal_Tar_Theta(void);		//计算到达目标点对应的各个关节角的大小
void PID_Col_Theta(void);		//PID控制关节打角
void Angle_To_Pwm(void);		//将关节角的值转变为目标PWM的值
void To_Center(void);				//当先PWM转为归中PWM
void To_Target(void);				//当前PWM转为目标PWM

double Distance_Measure(void);		//超声波测距函数

int main(void)
{
	u8 Receive_Right_Flag=FALSE;//接收正确标志位（FALSE为0，RIGHT为1）
	u8 i; 											//中间临时变量
	double distence;					//距离
	u8 instruction=0;						//指令，MOVE(0x01) / CATCH(0x02)
	u8 measure_count = 0;		//测距次数
	double Dis[10];
	u8 count = 0;					//有效测距次数
	
	/*****************硬件初始化&舵机角度归中**********************/
	delay_init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);
	LED_Init();
	Servor_GPIO_Config();
	Timer_Init();
	Triger_PB3_Init();
	TIM3_Cap_Init(0XFFFF,72-1);
	
	/********************开始***************************/
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
/***********************检测串口接收指令***************************/		
		if(Receive_Right_Flag == FALSE)//接收数据未校验通过，则进入此语句进行校验
		{
			if(Receive_Finish_Flag == FINISHED)//接受完成才进行校验
			{
				for(i=1;i<=Real_Length-2;i++)//求和，计算校验
				{
					Cheak=Cheak+Data_Buffer[i];			
				}
				if((Cheak == Data_Buffer[Real_Length-1])&&(Length==Real_Length))//校验正确
				{				
					Receive_Right_Flag=RIGHT;						//串口接收成功一条有效指令，退出while循环，直到执行完相应指令再准备接收下一条
					printf("check!\r\n");								//打印校验成功
					Receive_Finish_Flag = UNFINISHED;		//准备下一次接收
					//Receive_Right_Flag = FALSE;			//准备下一次进入循环
					
					/**************提取指令****************/
					instruction = Data_Buffer[1];
					/**************************************/
					
					Cheak = 0;
					Length = 0;
				}
				else//校验不正确
				{
					/*给主机返回信号*/
					/*缓存数据初始化*/
					Receive_Finish_Flag=FALSE;
					Cheak=0;//校验位清零	
					Length=0;//长度位清零
				}
			}
		}	

/******************************执行串口指令*******************************/
		switch(instruction)
		{
			case MOVE:			//01
			{
				/*运动到指定位置*/
				printf("move to the target!\r\n");	
				
				// 提取指令中的目标位置
				//x只能为负数
				target[0] = -((double)(Data_Buffer[2]&(0x7f)))/10;
				//y、z只能为正数
				target[1] = ((double)(Data_Buffer[3]&(0x7f)))/10;
				target[2] = ((double)(Data_Buffer[4]&(0x7f)))/10;
				
				Cal_Tar_Theta();		//计算关节角
				
				instruction = 0;							//清除指令
				Receive_Right_Flag = FALSE;		//允许接收下一条指令
				break;
			}
			
			case MEASURE:			//02
			{
				while(measure_count < 10)		//没检测够10次
				{
					Ultrasonic_Trig();	//触发超声波
					delay_ms(500);
					if(TIM3CH2_CAPTURE_STA&0X80)//成功捕获到了一次上升沿
					{
						distence = Distance_Measure();
						Dis[measure_count] = distence;
						printf("distence:%f cm\r\n",distence);//打印总的高点平时间		
					}
					measure_count++;
				}
					/*抓取物体*/
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
					distence = distence / count;	//计算平均距离
					count = 0;
					printf("mean distence:%f cm\r\n",distence);//打印平均距离
					
					/************计算物体坐标*************/
					//distence = distence - 4.8;
					distence = distence - 5;
					if (distence>=6 && distence <=20)
					{
						target[0] = -distence;				//设定x的坐标
						target[1] = target[2] = 0;		//y、z的坐标均为0
						Cal_Tar_Theta();
					}
					else
						printf("The object is in an unreasonable position!\r\n");
			
										
					measure_count = 0;
					instruction = 0;							//清除指令
					Receive_Right_Flag = FALSE;		//允许接收下一条指令
				
				break;
			}
			
			case CATCH:				//03
			{
				printf("catch!\r\n");
				theta[6] = 100;					//爪子合上
				
				instruction = 0;							//清除指令
				Receive_Right_Flag = FALSE;		//允许接收下一条指令			
				break;
			}
			case LOOSEN:					//04
			{
				printf("loosen!\r\n");
				theta[6] = 0;						//爪子松开
				
				instruction = 0;							//清除指令
				Receive_Right_Flag = FALSE;		//允许接收下一条指令			
				break;
			}
			
			case CATCH_UP:			//05
			{
				printf("catch up!\r\n");
				for (i=1;i<=4;i++)
				{
					theta[i] = 0;
				}
				
				instruction = 0;							//清除指令
				Receive_Right_Flag = FALSE;		//允许接收下一条指令			
				break;
			}
			
			case CENTER:					//06
			{
				/*归中*/
				printf("go to the center!\r\n");
				for (i=1;i<=6;i++)
				{
					theta[i] = 0;
				}
				instruction = 0;							//清除指令
				Receive_Right_Flag = FALSE;		//允许接收下一条指令
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
	temp*=65536;//溢出时间总和
	temp+=TIM3CH2_CAPTURE_VAL;//得到总的高电平时间
	distence = (double)temp * 0.0001 * 340 / 2;
	TIM3CH2_CAPTURE_STA=0;//开启下一次捕获
	return distence;
}

//计算到达目标点对应的各个关节角的大小
void Cal_Tar_Theta(void)
{
	int i;
	double x0,y0,beta,fai;
	theta[1] = atan(target[1] / target[0]);			//计算theta1
	
	//计算theta3
	x0 = pow((pow(target[0],2)+pow(target[1],2)),0.5);
	y0 = target[2];
	theta[3] = acos((pow(x0,2)+pow(y0,2)-pow(l1,2)-pow(l2,2))/(2*l1*l2));
	
	//计算theta2
	beta = atan(y0/x0);
	fai = acos((pow(x0,2)+pow(y0,2)+pow(l1,2)-pow(l2,2))/(2*l2*pow((pow(x0,2)+pow(y0,2)),0.5)));
	theta[2] = pi/2 - (beta + fai);
	
	//计算theta4
	//theta[4] = pi - theta[2] - theta[3];
	theta[4] = theta[3] + theta[2] - pi/2;
	
	for (i = 1; i <= 4; i++)
	{
		theta[i] = theta[i] / pi * 180;
	}
}

//PID控制关节打角
void PID_Col_Theta(void)
{
	double Kp = 0.04;
	int i;
	for (i = 0; i < 9; i++)
	{
		cul_theta[i] = cul_theta[i] + Kp*(theta[i] - cul_theta[i]);		
	}
}


//将关节角(角度)的值转变为目标PWM的值
void Angle_To_Pwm(void)
{
	int i;
	for(i = 0; i < 9; i++)
	{
		if(i == 3)
			Target_PWM[i] = 1500 - (int)(cul_theta[i] * 7.41);		//确保正方向
		else
			Target_PWM[i] = 1500 + (int)(cul_theta[i] * 7.41);
	}
}

//当先PWM转为归中PWM
void To_Center(void)
{
	int i;
	for (i = 0; i < 9; i++)
	{
		CPWM[i] = Center_PWM[i];
	}
}

//当前PWM转为目标PWM
void To_Target(void)				
{
	int i;
	for (i = 0; i < 9; i++)
	{
		CPWM[i] = Target_PWM[i];
	}
}



