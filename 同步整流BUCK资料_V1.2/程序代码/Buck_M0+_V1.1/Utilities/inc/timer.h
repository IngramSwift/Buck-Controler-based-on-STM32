#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f0xx.h"

#define SYSTEMP_COLOCK			48000000u	//Hz.ϵͳʱ��
#define TIM3_COLOCK				10000		//Hz.��ʱ��ʱ��
#define TIM3_xTIMER				0.002f		//S.��ʱʱ��
#define TIM3_PRESCALER			(uint16_t)(SYSTEMP_COLOCK/TIM3_COLOCK - 1)	//Ԥ��Ƶֵ
#define TIM3_PERIOD				(uint16_t)(TIM3_xTIMER*TIM3_COLOCK)			//����ֵ
#define TIM3_CHECK_PERIOD(a)	a=(a<1)?(0):(a-1) //��ֹ�������ô���


#define TIM14_COLOCK			1000000		//Hz.��ʱ��ʱ��
#define TIM14_xTIMER			0.0001f	//S.��ʱʱ��
#define TIM14_PRESCALER			(uint16_t)(SYSTEMP_COLOCK/TIM14_COLOCK - 1)	//Ԥ��Ƶֵ
#define TIM14_PERIOD			(uint16_t)(TIM14_xTIMER*TIM14_COLOCK)		//����ֵ
#define TIM14_CHECK_PERIOD(a)	a=(a<1)?(0):(a-1) //��ֹ�������ô���


void TIM3_Config(void);
void TIM14_Config(void);


#endif















