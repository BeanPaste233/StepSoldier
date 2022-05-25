#include "timer.h"
#include "delay.h"


void DCMotor_Init()
{
	DCMotorTimer_Init();
}
void SteerMotor_Init()
{
	SteerTimer_Init();
}
void PullBulletMotor_Init()
{
	PullBulletMotorTimer_Init();

}
void PullMotor_Back(uint16_t speed)
{
	//关闭通道3 开启通道1 反转
	if(speed>20000||speed<10000) return;
	TIM_CCxCmd(TIM3,TIM_Channel_3,TIM_CCx_Disable);
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);
	delay_ms(500);
	TIM_CCxCmd(TIM3,TIM_Channel_1,TIM_CCx_Enable);
	TIM_SetCompare1(TIM3,speed);
	//TIM_SetCompare1(TIM3,10000);
	//TIM_SetCompare3(TIM3,0);
}
void PullMotor_Forward(uint16_t speed)
{
	//关闭通道1 开启通道3 正转
	if(speed>20000||speed<10000) return;
	
	
	//TIM_SetCompare1(TIM3,0);
	TIM_CCxCmd(TIM3,TIM_Channel_1,TIM_CCx_Disable);
  GPIO_ResetBits(GPIOA,GPIO_Pin_6);
	delay_ms(500);
	TIM_CCxCmd(TIM3,TIM_Channel_3,TIM_CCx_Enable);
	TIM_SetCompare3(TIM3,speed);
}
void AllMotor_Stop(void)
{
	TIM_SetCompare1(TIM1,0);
	TIM_SetCompare2(TIM1,0);
}
void RightMotor_Forward(uint16_t speed)
{
	TIM_SetCompare1(TIM1,0);
	TIM_SetCompare2(TIM1,speed);
}

void RightMotor_Reverse(uint16_t speed)
{
	TIM_SetCompare1(TIM1,speed);
	TIM_SetCompare2(TIM1,0);
}

void LeftMotor_Forward(uint16_t speed)
{
	TIM_SetCompare3(TIM1,speed);
	TIM_SetCompare4(TIM1,0);
}
void LeftMotor_Reverse(uint16_t speed)
{
	TIM_SetCompare3(TIM1,0);
	TIM_SetCompare4(TIM1,speed);
}
void RubESCUnlock(void)//摩擦轮电调解锁
{
		TIM_SetCompare2(TIM2,1000);
		TIM_SetCompare3(TIM2,1000);
}
void RubMotorInit()//摩擦轮电机初始化
{
	RubSteelTimer_Init();
	RubESCUnlock();
}
void SetLeftSpeed(uint16_t speed)
{
	uint16_t final=(uint16_t)((float)speed/100*1000)+1000;
	TIM_SetCompare2(TIM2,final);
}
void SetRightSpeed(uint16_t speed)
{
	uint16_t final=(uint16_t)((float)speed/100*1000)+1000;
	TIM_SetCompare3(TIM2,final);

}
void SetXAngle(uint16_t angle)//X轴舵机转向
{
	SteerTimer_SetAngle(TIM3,4,angle);
}
void SetYAngle(uint16_t angle)//Y轴舵机转向
{
	SteerTimer_SetAngle(TIM2,1,angle);
}
void SetFrontAngle(uint16_t angle)//前轮舵机转向
{
	SteerTimer_SetAngle(SteerTim,1,angle);
}