/********************************************************************
pwm.c

Copyright (c) 2014, Jonathan Nutzmann

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
********************************************************************/

/****************************************************************************
 * Includes
 ***************************************************************************/

#include <stm32f4xx.h>

#include "FreeRTOS.h"
#include "task.h"

#include "d1k.h"
#include "smarts_leds.h"
#include "halls.h"

#include "pwm.h"
#include "stdio.h"

#include "adc.h"

#include "system_cfg.h"
#include "fault.h"
#include "math_limits.h"

#define PWM_AN	GPIO_Pin_8
#define PWM_A	GPIO_Pin_9
#define PWM_BN	GPIO_Pin_10
#define PWM_B	GPIO_Pin_11
#define PWM_CN	GPIO_Pin_12
#define PWM_C	GPIO_Pin_13

#define PWM_GPIO GPIOE

#define MINIMUM_DUTY_CYCLE (0.0f)
#define MAXIMUM_DUTY_CYCLE (0.96f)

static uint32 sPWMperiod = 0;
static ControlLoopFxn sControlLoopFxn = NULL;

/**
 * Initializes the PWM output of the controller.
 * @param frequency - Frequency (in Hz) of the switching.
 */
void pwm_Init( uint32 frequency )
{
	// To calculate the period, we multiply the clock frequency by two
	// (there is a scaler for this timer), then also multiply the frequency
	// by two (because it's center aligned), so they cancel each other out.
	sPWMperiod = (SYS_APB2_HZ / frequency );

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	GPIO_PinAFConfig(PWM_GPIO, GPIO_PinSource8, GPIO_AF_TIM1);
	GPIO_PinAFConfig(PWM_GPIO, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(PWM_GPIO, GPIO_PinSource10, GPIO_AF_TIM1);
	GPIO_PinAFConfig(PWM_GPIO, GPIO_PinSource11, GPIO_AF_TIM1);
	GPIO_PinAFConfig(PWM_GPIO, GPIO_PinSource12, GPIO_AF_TIM1);
	GPIO_PinAFConfig(PWM_GPIO, GPIO_PinSource13, GPIO_AF_TIM1);

	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = PWM_AN | PWM_A | PWM_BN | PWM_B | PWM_CN | PWM_C;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(PWM_GPIO, &gpio);

	TIM_DeInit(TIM1);

	TIM_TimeBaseInitTypeDef tb;
	TIM_TimeBaseStructInit(&tb);
	tb.TIM_ClockDivision = TIM_CKD_DIV1;
	tb.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	tb.TIM_Period = sPWMperiod;
	TIM_TimeBaseInit(TIM1, &tb);

	TIM_OCInitTypeDef oc;
	TIM_OCStructInit(&oc);
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
	oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	oc.TIM_OCPolarity = TIM_OCPolarity_High;
	oc.TIM_OCNPolarity = TIM_OCNPolarity_High;
	oc.TIM_OCMode = TIM_OCMode_PWM1;
	oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OC1Init(TIM1, &oc);
	TIM_OC2Init(TIM1, &oc);
	TIM_OC3Init(TIM1, &oc);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_CCPreloadControl(TIM1, DISABLE);

	TIM_BDTRInitTypeDef bdtr;
	TIM_BDTRStructInit(&bdtr);
	bdtr.TIM_DeadTime = 40; // ???ns
	bdtr.TIM_Break = TIM_Break_Disable; // TODO: enable
	bdtr.TIM_BreakPolarity = TIM_BreakPolarity_Low;
	bdtr.TIM_OSSIState = TIM_OSSIState_Enable;
	bdtr.TIM_OSSRState = TIM_OSSRState_Enable;

	//bdtr.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;

	TIM_BDTRConfig(TIM1, &bdtr);

	TIM_Cmd(TIM1, ENABLE);

	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);

	NVIC_InitTypeDef nvicInit;
	nvicInit.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	nvicInit.NVIC_IRQChannelCmd = ENABLE;
	nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
	nvicInit.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init( &nvicInit );

	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}


/**
 * Disables all channels on the motor controller.
 */
void pwm_EStop(void)
{
	TIM_CtrlPWMOutputs(TIM1, DISABLE);
}

/**
 * Configures whether the PWM channel is active.  If enabled, duty cycle is set by
 * pwm_SetDutyCycles.  If disabled, all gates will be low and the FETs will be floating.
 * @param a - Whether channel A is enabled.
 * @param b - Whether channel B is enabled.
 * @param c - Whether channel C is enabled.
 */
void pwm_SetPWMOutputEnable( FunctionalState a, FunctionalState b, FunctionalState c )
{
	if ( a == ENABLE )
	{
		TIM_CCxNCmd(TIM1,TIM_Channel_1,TIM_CCxN_Enable);
	}
	else
	{
		TIM_CCxNCmd(TIM1,TIM_Channel_1,TIM_CCxN_Disable);
		TIM_SetCompare1(TIM1, 0);
	}

	if ( b == ENABLE)
	{
		TIM_CCxNCmd(TIM1,TIM_Channel_2,TIM_CCxN_Enable);
	}
	else
	{
		TIM_CCxNCmd(TIM1,TIM_Channel_2,TIM_CCxN_Disable);
		TIM_SetCompare2(TIM1, 0);
	}

	if ( c == ENABLE )
	{
		TIM_CCxNCmd(TIM1,TIM_Channel_3,TIM_CCxN_Enable);
	}
	else
	{
		TIM_CCxNCmd(TIM1,TIM_Channel_3,TIM_CCxN_Disable);
		TIM_SetCompare3(TIM1, 0);
	}
}

/**
 * Sets the PWM duty cycles output of the motor controller.  Values between
 * 0 and 1 are mapped to 0 to 100% duty cycle.
 * @param a - Phase A duty cycle.
 * @param b - Phase B duty cycle.
 * @param c - Phase C duty cycle.
 */
void pwm_SetDutyCycles(float a, float b, float c)
{
	a = limitf32(a,0.97f,0);
	b = limitf32(b,0.97f,0);
	c = limitf32(c,0.97f,0);

	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

	TIM_SetCompare1(TIM1, (uint32_t)(limitf32(a, MAXIMUM_DUTY_CYCLE, MINIMUM_DUTY_CYCLE) * sPWMperiod));

	TIM_SetCompare2(TIM1, (uint32_t)(limitf32(b, MAXIMUM_DUTY_CYCLE, MINIMUM_DUTY_CYCLE) * sPWMperiod));

	TIM_SetCompare3(TIM1, (uint32_t)(limitf32(c, MAXIMUM_DUTY_CYCLE, MINIMUM_DUTY_CYCLE) * sPWMperiod));
}

/**
 * Registers the control loop that will be called every time the PWM interrut
 * occurs.
 * @param fxn - Function to be called.
 */
void pwm_RegisterControlLoopFxn ( ControlLoopFxn fxn )
{
	sControlLoopFxn = fxn;
}

/**
 * PWM Interrupt handler.
 */
void TIM1_UP_TIM10_IRQHandler ( void )
{
	if ( TIM_GetITStatus(TIM1,TIM_IT_Update) )
	{
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
		sControlLoopFxn();
	}
}
