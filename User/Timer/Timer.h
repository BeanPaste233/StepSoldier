#ifndef __TIMER_H
#define __TIMER_H
#include "stm32f10x.h"


#define SteerTim              TIM3
#define SteerTim_GPIO         GPIOA
#define SteerTim_GPIO_Pin     GPIO_Pin_6
#define SteerTim_RCC_CMD      RCC_APB1Periph_TIM3
#define SteerGPIO_RCC_CMD     RCC_APB2Periph_GPIOA
#define SteerGPIO_Mode        GPIO_Mode_AF_PP
#define SteerGPIO_Speed       GPIO_Speed_50MHz
void SteerTimer_SetAngle(TIM_TypeDef *timx,uint8_t channel,uint16_t angle);
void SteerTimer_Init(void);
void DCMotorTimer_Init(void);
void RubSteelTimer_Init(void);
void PullBulletMotorTimer_Init(void);
#endif