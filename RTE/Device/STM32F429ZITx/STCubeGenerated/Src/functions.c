#include "string.h"
#include "usart.h"
#include "FreeRTOS.h"
#include <stdlib.h>  
#include "task.h"
#include "stdbool.h"
#include <math.h>
#include "tim.h"


 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SM_Move (int32_t speed, int8_t turn, bool enable)
{
	if (enable)
	{
		HAL_GPIO_WritePin(SM1En_GPIO_Port, SM1En_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SM2En_GPIO_Port, SM2En_Pin, GPIO_PIN_SET);
		
		if (speed==0)
		{
			__HAL_RCC_TIM11_CLK_DISABLE();
			__HAL_RCC_TIM13_CLK_DISABLE();

			TIM13->CNT=0;
			TIM11->CNT=0;
		}
		else
		{
			__HAL_RCC_TIM13_CLK_ENABLE();
			__HAL_RCC_TIM11_CLK_ENABLE();
			
		//SM1 CONFIGURATION/////////////////////////////////////////////////////////////////////////
			int32_t speedSM1=speed+(10*turn);
			if (speedSM1>0)
			{
				HAL_GPIO_WritePin(SM1Dir_GPIO_Port, SM1Dir_Pin, GPIO_PIN_RESET);		
			}
			else if (speedSM1<0)
			{
				HAL_GPIO_WritePin(SM1Dir_GPIO_Port, SM1Dir_Pin, GPIO_PIN_SET);
				speedSM1=abs(speedSM1);
			}

			if (speedSM1>2340)
			{
					speedSM1=2340;
			}
			uint16_t timerSM1value = 32768/speedSM1;
			
		//SM2 CONFIGURATION/////////////////////////////////////////////////////////////////////////
			int32_t speedSM2=speed-(10*turn);
			if (speedSM2>0)
			{
				HAL_GPIO_WritePin(SM2Dir_GPIO_Port, SM2Dir_Pin, GPIO_PIN_SET);
				
			}
			else if (speedSM2<0)
			{
				HAL_GPIO_WritePin(SM2Dir_GPIO_Port, SM2Dir_Pin, GPIO_PIN_RESET);
				speedSM2=abs(speedSM2);
			}

			if (speedSM2>2340)
			{
					speedSM2=2340;
			}
			uint16_t timerSM2value = 32768/speedSM2;
			
		//SM1 AND SM2 CONFIGURATION SEND TO STEPPER MOTORS////////////////////////////////////////////					
			if(TIM13->ARR>timerSM1value)
			{
				TIM13->CNT=0;
			}
			if(TIM11->ARR>timerSM2value)
			{
				TIM11->CNT=0;		
			}
			TIM13->ARR=timerSM1value;
			TIM11->ARR=timerSM2value;
	  }	
	}
	else
	{
	 HAL_GPIO_WritePin(SM1En_GPIO_Port, SM1En_Pin, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(SM2En_GPIO_Port, SM2En_Pin, GPIO_PIN_RESET);
	}
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t sign(float x)
{
	if (x>=0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
