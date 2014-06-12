/********************************************************************
diagnostics.c

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

#include "FreeRTOS.h"
#include "task.h"

#include "d1k_led.h"
#include "smarts_leds.h"

#include "diagnostics.h"
#include "fan.h"
#include "motor.h"
#include "fullCAN.h"
#include "string.h"


/****************************************************************************
 * Definitions
 ***************************************************************************/

#define DIAGNOSTICS_TASK_FREQUENCY	(5)

/****************************************************************************
 * Global Variables
 ***************************************************************************/

xTaskHandle diagnosticsTaskHandle;

static uint32 errorsPresent = 0;
static uint32 warningsPresent = 0;

/****************************************************************************
 * Prototypes
 ***************************************************************************/

static void diag_Task( void * pvParameters );

/****************************************************************************
 * Public Functions
 ***************************************************************************/

/**
 * Initializes the diagnostics handler.
 */
void diag_Init( void )
{
	xTaskCreate(diag_Task, "DIAGNOS", 1024, NULL, 3, &diagnosticsTaskHandle);

	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* Configure the GPIO_LED pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

bool diag_GetHardwareOvercurrent( void )
{
	return !GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1);
}

void diag_ThrowError( ErrorEnum_t error )
{
	errorsPresent |= (uint32) (0x1 << error);
}

void diag_ClearError( ErrorEnum_t error )
{
	errorsPresent &= ~((uint32) (0x1 << error));
}

bool diag_GetErrorState( ErrorEnum_t error )
{
	return errorsPresent & (0x1 << error);
}

uint32 diag_GetErrorBitmask ( void )
{
	return errorsPresent;
}

void diag_ThrowWarning( WarningEnum_t warning )
{
	warningsPresent |= (uint32) (0x1 << warning);
}

void diag_ClearWarning( WarningEnum_t warning )
{
	warningsPresent &= ~((uint32) (0x1 << warning));
}

bool diag_GetWarningState( WarningEnum_t warning )
{
	return warningsPresent & (0x1 << warning);
}

uint32 diag_GetWarningBitmask( void )
{
	return warningsPresent;
}

/****************************************************************************
 * Private Functions
 ***************************************************************************/

#define BITMASK(e,a) (a << e)

static void diag_Task( void * pvParameters )
{
	portTickType xLastWakeTime = xTaskGetTickCount();

	CanTxMsg msg;

	msg.RTR = CAN_RTR_Data;
	msg.DLC = 8;

	while (1)
	{
		if (warningsPresent) {
			d1k_LED_On(WARNING_LED);
		} else {
			d1k_LED_Off(WARNING_LED);
		}

		if (errorsPresent & (
				  BITMASK(ERROR_MOTOR_HALLS,1)
				| BITMASK(ERROR_MOTOR_NOT_PRESENT,1)
				| BITMASK(ERROR_MOTOR_OVERTEMP,1)
		)) {
			d1k_LED_On(MOT_ERROR_LED);
		} else {
			d1k_LED_Off(MOT_ERROR_LED);
		}

		if (errorsPresent & (
				  BITMASK(ERROR_CURRENT_HARDWARE_FAULT,1)
				| BITMASK(ERROR_CURRENT_SOFTWARE_FAULT,1)
				| BITMASK(ERROR_CURRENT_NONE,1)
				| BITMASK(ERROR_CURRENT_IMBALANCE,1)
		)) {
			d1k_LED_On(I_ERROR_LED);
		} else {
			d1k_LED_Off(I_ERROR_LED);
		}

		if (errorsPresent & (
				  BITMASK(ERROR_VOLTAGE_HIGH_PHASE_A,1)
				| BITMASK(ERROR_VOLTAGE_HIGH_PHASE_B,1)
				| BITMASK(ERROR_VOLTAGE_HIGH_PHASE_C,1)
				| BITMASK(ERROR_VOLTAGE_HIGH_BUS,1)
				| BITMASK(ERROR_VOLTAGE_HIGH_15,1)
				| BITMASK(ERROR_VOLTAGE_LOW_15,1)
		)) {
			d1k_LED_On(V_ERROR_LED);
		} else {
			d1k_LED_Off(V_ERROR_LED);
		}

		if (errorsPresent & (
						  BITMASK(ERROR_OVERTEMP_A,1)
						| BITMASK(ERROR_OVERTEMP_AN,1)
						| BITMASK(ERROR_OVERTEMP_B,1)
						| BITMASK(ERROR_OVERTEMP_BN,1)
						| BITMASK(ERROR_OVERTEMP_C,1)
						| BITMASK(ERROR_OVERTEMP_CN,1)
						| BITMASK(ERROR_OVERTEMP_AMB,1)
						| BITMASK(ERROR_OVERTEMP_HS,1)
				)) {
					d1k_LED_On(GEN_ERROR_LED);
				} else {
					d1k_LED_Off(GEN_ERROR_LED);
				}


		memcpy(&msg.Data[0],&errorsPresent,sizeof(errorsPresent));
		memcpy(&msg.Data[4],&warningsPresent,sizeof(warningsPresent));

		fullCAN_SendPacket(&msg,FULLCAN_IDBASE_EVENTS);

		vTaskDelayUntil( &xLastWakeTime, configTICK_RATE_HZ/DIAGNOSTICS_TASK_FREQUENCY);
	}
}
