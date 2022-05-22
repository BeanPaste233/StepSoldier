#include "timer.h"



//ǰ�ֶ����ʱ����ʼ��
void SteerTimer_Init(void)
{
	//����ʱ��
	RCC_APB1PeriphClockCmd(SteerTim_RCC_CMD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	RCC_APB2PeriphClockCmd(SteerGPIO_RCC_CMD,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	//��ʼ��GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=SteerGPIO_Mode;
	GPIO_InitStructure.GPIO_Pin=SteerTim_GPIO_Pin;
	GPIO_InitStructure.GPIO_Speed=SteerGPIO_Speed;
	GPIO_Init(SteerTim_GPIO,&GPIO_InitStructure);
	
	
	//X����PWM-GPIO��
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	//Y����PWM-GPIO��
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//����ʱ��Դ
	TIM_InternalClockConfig(SteerTim);
	TIM_InternalClockConfig(TIM2);
	
	//����ʱ����Ԫ
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period=20000-1;
	TIM_TimeBaseInitStructure.TIM_Prescaler=72-1;
	TIM_TimeBaseInit(SteerTim,&TIM_TimeBaseInitStructure);
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	//��������Ƚ�
	TIM_OCInitTypeDef OC_InitStructure;
	OC_InitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	OC_InitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	OC_InitStructure.TIM_OutputState=TIM_OutputState_Enable;
	OC_InitStructure.TIM_Pulse=500;
	TIM_OC1Init(SteerTim,&OC_InitStructure);
	TIM_OC4Init(TIM3,&OC_InitStructure);
	TIM_OC1Init(TIM2,&OC_InitStructure);

	TIM_Cmd(SteerTim,ENABLE);
	TIM_Cmd(TIM2,ENABLE);
}

void SteerTimer_SetAngle(TIM_TypeDef *timx,uint8_t channel,uint16_t angle)
{
	uint16_t count=((float)angle/180)*2000+500;
	switch(channel)
	{
		case 1:TIM_SetCompare1(timx,count);break;
		case 2:TIM_SetCompare2(timx,count);break;
		case 3:TIM_SetCompare3(timx,count);break;
		case 4:TIM_SetCompare4(timx,count);break;
	}
	//TIM_SetCompare1(timx,count);
}


//����ֱ�������ʱ����ʼ��
void DCMotorTimer_Init(void)
{
	GPIO_InitTypeDef gpioInitStructure;
	TIM_TimeBaseInitTypeDef tim1TimeBaseStructure;

	//����ʱ��

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	//����GPIO
	gpioInitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	gpioInitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	gpioInitStructure.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
	GPIO_Init(GPIOA,&gpioInitStructure);

	
	//����TIM3
	
	//����ʱ��
	
	tim1TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	tim1TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	tim1TimeBaseStructure.TIM_Period=20000-1;
	tim1TimeBaseStructure.TIM_Prescaler=72-1;
	tim1TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM1,&tim1TimeBaseStructure);
	
	//ѡ��ʱ��Դ 
	TIM_InternalClockConfig(TIM1);
	
	
	//��������Ƚ�
	TIM_OCInitTypeDef timOCInitStructure;
	timOCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	timOCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	timOCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	timOCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Set;
	timOCInitStructure.TIM_OCNIdleState=TIM_OCNIdleState_Reset;
	timOCInitStructure.TIM_OCNPolarity=TIM_OCNPolarity_Low;
	timOCInitStructure.TIM_OutputNState=TIM_OutputNState_Disable;
	
	
	TIM_OC1Init(TIM1,&timOCInitStructure);
	TIM_OC2Init(TIM1,&timOCInitStructure);
	TIM_OC3Init(TIM1,&timOCInitStructure);
	TIM_OC4Init(TIM1,&timOCInitStructure);
	
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_Cmd(TIM1,ENABLE);
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
}

//Ħ����ʱ�ӳ�ʼ��
void RubSteelTimer_Init(void)
{
	//����ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	//��ʼ��GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//����ʱ��Դ
	TIM_InternalClockConfig(TIM2);
	
	//����ʱ����Ԫ
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period=2000-1;
	TIM_TimeBaseInitStructure.TIM_Prescaler=72-1;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	//��������Ƚ�
	TIM_OCInitTypeDef OC_InitStructure;
	OC_InitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	OC_InitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	OC_InitStructure.TIM_OutputState=TIM_OutputState_Enable;
	OC_InitStructure.TIM_Pulse=2000;
	TIM_OC2Init(TIM2,&OC_InitStructure);
	TIM_OC3Init(TIM2,&OC_InitStructure);
	

	TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM2,TIM_OCPreload_Enable);

	TIM_Cmd(TIM2,ENABLE);

}


//���������ʱ����ʼ��
void PullBulletMotorTimer_Init(void)
{
	//����ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	//��ʼ��GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	//����ʱ��Դ
	TIM_InternalClockConfig(TIM3);
	
	//����ʱ����Ԫ
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period=20000-1;
	TIM_TimeBaseInitStructure.TIM_Prescaler=72-1;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	//��������Ƚ�
	TIM_OCInitTypeDef OC_InitStructure;
	OC_InitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	OC_InitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	OC_InitStructure.TIM_OutputState=TIM_OutputState_Enable;
	
	
	TIM_OC1Init(TIM3,&OC_InitStructure);
	TIM_OC3Init(TIM3,&OC_InitStructure);
	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_Cmd(TIM3,ENABLE);



}

