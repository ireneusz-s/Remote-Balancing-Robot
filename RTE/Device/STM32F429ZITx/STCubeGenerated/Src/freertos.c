/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "gpio.h"
#include "usart.h"
#include "cli.h"
#include "commands.h"
#include "functions.h"
#include "math.h"
#include "adc.h"
#include "IS_MPU6050.h"
#include "IS_complementaryFilter.h"
#include "IS_FIR.h"
#include "IS_PID.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId GreenLedBlinkHandle;
osThreadId taskCLIHandle;
osThreadId DataProcessHandle;
osThreadId DataSendLVHandle;
osThreadId AdcHandle;

/* USER CODE BEGIN Variables */
extern ADC_HandleTypeDef hadc1;
uint32_t ValueADC1=0; //variable stores adv value (battery)

//GLOBAL VARIABLES
bool DeviceOn=false;
uint8_t SampleRate=2;
int32_t SM_Move_Var =0;
int32_t SM_Turn_Var =0;

//GLOBAL STRUCTURES
IS_MPU6050_struct MPU6050_Data;
IS_CompFilter_struct CompFilter_Data; 
IS_PID_struct PID1_Data;
IS_PID_struct PID2_Data;


int32_t PID_outputFiltered=0;

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void GreenLedBlinkFunction(void const * argument);
void CLIFunction(void const * argument);
void DataProcessFunction(void const * argument);
void DataSendLVFunction(void const * argument);
void AdcTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of GreenLedBlink */
  osThreadDef(GreenLedBlink, GreenLedBlinkFunction, osPriorityLow, 0, 128);
  GreenLedBlinkHandle = osThreadCreate(osThread(GreenLedBlink), NULL);

  /* definition and creation of taskCLI */
  osThreadDef(taskCLI, CLIFunction, osPriorityAboveNormal, 0, 256);
  taskCLIHandle = osThreadCreate(osThread(taskCLI), NULL);

  /* definition and creation of DataProcess */
  osThreadDef(DataProcess, DataProcessFunction, osPriorityRealtime, 0, 2048);
  DataProcessHandle = osThreadCreate(osThread(DataProcess), NULL);

  /* definition and creation of DataSendLV */
  osThreadDef(DataSendLV, DataSendLVFunction, osPriorityNormal, 0, 512);
  DataSendLVHandle = osThreadCreate(osThread(DataSendLV), NULL);

  /* definition and creation of Adc */
  osThreadDef(Adc, AdcTask, osPriorityBelowNormal, 0, 128);
  AdcHandle = osThreadCreate(osThread(Adc), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	
	//vTaskSuspend(DataProcessHandle);       //wylaczam odpowiednie taski
	vTaskSuspend(DataSendLVHandle);
	vTaskSuspend(AdcHandle);
	
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* GreenLedBlinkFunction function */
void GreenLedBlinkFunction(void const * argument)
{
  /* USER CODE BEGIN GreenLedBlinkFunction */

	
  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
		USART_WriteString("test\n");
		osDelay(500);
  }
  /* USER CODE END GreenLedBlinkFunction */
}

/* CLIFunction function */
void CLIFunction(void const * argument)
{
  /* USER CODE BEGIN CLIFunction */
	
				CLI_CommandItem item_LED = { .callback = commandLED,
                                  .commandName = "LED",
                                  .description = "steruje dioda czerwona"};
				
				CLI_CommandItem item_SM_Move = { .callback = commandSM_Move,
                                  .commandName = "SM_Move",
                                  .description = "steruje silnikam krokowymi"};
				
				CLI_CommandItem item_SM_Turn = { .callback = commandSM_Turn,
                                  .commandName = "SM_Turn",
                                  .description = "Skret silnikow krokowych"};
				
        CLI_CommandItem item_PID_Kp = { .callback = commandPID_Kp,
                                  .commandName = "PID_Kp",
                                  .description = "Zmienia nastawe P regulatora PID"};

        CLI_CommandItem item_PID_Ki = { .callback = commandPID_Ki,
                                  .commandName = "PID_Ki",
                                  .description = "Zmienia nastawe I regulatora PID"};

        CLI_CommandItem item_PID_Kd = { .callback = commandPID_Kd,
                                  .commandName = "PID_Kd",
                                  .description = "Zmienia nastawe D regulatora PID"};

        CLI_CommandItem item_PID2_Kp = { .callback = commandPID2_Kp,
                                  .commandName = "PID2_Kp",
                                  .description = "Zmienia nastawe P regulatora PID2"};

        CLI_CommandItem item_PID2_Ki = { .callback = commandPID2_Ki,
                                  .commandName = "PID2_Ki",
                                  .description = "Zmienia nastawe I regulatora PID2"};

        CLI_CommandItem item_PID2_Kd = { .callback = commandPID2_Kd,
                                  .commandName = "PID2_Kd",
                                  .description = "Zmienia nastawe D regulatora PID2"};
								
        CLI_CommandItem item_Offset_Ang = { .callback = commandOffset_Ang,
                                  .commandName = "Offset_Ang",
                                  .description = "Kalibruje kat przy ktorym robot utrzymuje rownowage"};
								
        CLI_CommandItem item_CF_A = { .callback = commandCF_A,
                                  .commandName = "CF_A",
                                  .description = "Zmienia nastawe A filtra komplementarnego"};
												
        CLI_CommandItem item_SampleRate = { .callback = commandSampleRate,
                                  .commandName = "SampleRate",
                                  .description = "Zmienia nastawe iSampleRate"};
				
        CLI_CommandItem item_DeviceOn = { .callback = commandDeviceOn,
                                  .commandName = "DeviceOn",
                                  .description = "Wlacza lub wylacza robota"};
				
        CLI_CommandItem item_PID_AntiWindup = { .callback = commandPID_AntiWindup,
                                  .commandName = "PID_AntiWindup",
                                  .description = "Zmienia zakres pracy Filtru AntiWindup czlonu calkojucego PID"};
												
        CLI_CommandItem item_PID2_AntiWindup = { .callback = commandPID2_AntiWindup,
                                  .commandName = "PID2_AntiWindup",
                                  .description = "Zmienia zakres pracy Filtru AntiWindup czlonu calkojucego PID2"};
///////////////////
																	
				if(CLI_AddCommand(&item_LED) == false){
            USART_WriteString("ERROR in adding new item.\n\r");
        }
				if(CLI_AddCommand(&item_SM_Move) == false){
            USART_WriteString("ERROR in adding new item.\n\r");
        }
				if(CLI_AddCommand(&item_SM_Turn) == false){
            USART_WriteString("ERROR in adding new item.\n\r");
        }
				if(CLI_AddCommand(&item_PID_Kp) == false){
            USART_WriteString("ERROR in adding new item.\n\r");
        }
				if(CLI_AddCommand(&item_PID_Ki) == false){
            USART_WriteString("ERROR in adding new item.\n\r");
        }
				if(CLI_AddCommand(&item_PID_Kd) == false){
            USART_WriteString("ERROR in adding new item.\n\r");
        }
				if(CLI_AddCommand(&item_PID2_Kp) == false){
            USART_WriteString("ERROR in adding new item.\n\r");
        }
				if(CLI_AddCommand(&item_PID2_Ki) == false){
            USART_WriteString("ERROR in adding new item.\n\r");
        }
				if(CLI_AddCommand(&item_PID2_Kd) == false){
            USART_WriteString("ERROR in adding new item.\n\r");
        }
				if(CLI_AddCommand(&item_Offset_Ang) == false){
            USART_WriteString("ERROR in adding new item.\n\r");
        }		
				if(CLI_AddCommand(&item_CF_A) == false){
            USART_WriteString("ERROR in adding new item.\n\r");
        }
				if(CLI_AddCommand(&item_SampleRate) == false){
            USART_WriteString("ERROR in adding new item.\n\r");
        }
				if(CLI_AddCommand(&item_DeviceOn) == false){
            USART_WriteString("ERROR in adding new item.\n\r");
        }
				if(CLI_AddCommand(&item_PID_AntiWindup) == false){
            USART_WriteString("ERROR in adding new item.\n\r");
        }
				if(CLI_AddCommand(&item_PID2_AntiWindup) == false){
            USART_WriteString("ERROR in adding new item.\n\r");
        }
  /* Infinite loop *///
  for(;;)
  {
		CLI_Proc();
    osDelay(1);
  }
  /* USER CODE END CLIFunction */
}

/* DataProcessFunction function */
void DataProcessFunction(void const * argument)
{
  /* USER CODE BEGIN DataProcessFunction */
		
	 //Initialize 
		IS_MPU6050_Initialize(&MPU6050_Data, MPU6050_Accelerometer_2G, MPU6050_Gyroscope_250s);
		IS_CompFilter_initialize(&CompFilter_Data);
		IS_PID_initialize(&PID1_Data);
		IS_PID_initialize(&PID2_Data);
	
		
		TickType_t xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();

	
		float dt=(float)SampleRate/1000;


//MPU6050 important Data	
		static float ay1=0;
		static float az1=0;
		static float gx1=0;
	
//PID1_CONFIGURATION
			PID1_Data.PID_KP=800;
			PID1_Data.PID_KI=0;
			PID1_Data.PID_KD=6;
			PID1_Data.PID_AntiWindup=100;
			PID1_Data.PID_outLimit=2340;
			
//PID2_CONFIGURATION
			PID2_Data.PID_KP=0.3;
			PID2_Data.PID_KI=0.003;
			PID2_Data.PID_KD=0;
			PID2_Data.PID_AntiWindup=300;
			PID2_Data.PID_outLimit=5;
			

	  /* Infinite loop */
  for(;;) 
  {
	  vTaskDelayUntil(&xLastWakeTime,SampleRate);

//Read data from MPU6050 and save to varibles//////////////////////////////////////////////////////////////////////////////
		IS_MPU6050_ReadALL(&MPU6050_Data);
		
		ay1 = MPU6050_Data.Accelerometer_Y*2.0f/32768.0f;
		az1 = MPU6050_Data.Accelerometer_Z*2.0f/32768.0f;
		gx1 = -1*MPU6050_Data.Gyroscope_X*250.0f/32768.0f-3.80f; 

	
//COMPLEMENTARY FILTER////////////////////////////////////////////////////////////////////////////////////////////////////
	  IS_CompFilter_calculate(&CompFilter_Data, az1, ay1, gx1, dt);
	
		
//PID1///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	  IS_PID_calculate(&PID1_Data, PID2_Data.PID_output-CompFilter_Data.roll, dt);
		
	  PID_outputFiltered=IS_FIR_calculate(PID1_Data.PID_output);			
		
		
//PID2//////////////////////////////////////////////////////////////////////////////////////////////////////////////////				
	  IS_PID_calculate3(&PID2_Data, (SM_Move_Var*10)+PID_outputFiltered, dt);


//PID OUTPUT TO MOTORS/////////////////////////////////////////////////////////////////////////////////////////////////////
	  if (DeviceOn && fabs(CompFilter_Data.roll)<50)					 //TURN ON MOTORS
 	  {
	  	SM_Move(PID1_Data.PID_output,SM_Turn_Var,true);
	  }
	  else
    {
	  	SM_Move(0,0,false);
	  }

  }
  /* USER CODE END DataProcessFunction */
}

/* DataSendLVFunction function */
void DataSendLVFunction(void const * argument)
{
  /* USER CODE BEGIN DataSendLVFunction */
	
	static char comp_sprintf_buffer[256]; //BUFFER TO SEND LABVIEW
	static char pid_sprintf_buffer[256];  //BUFFER TO SEND LABVIEW
	
  /* Infinite loop */
  for(;;)
  {
		sprintf(comp_sprintf_buffer,"ch1:%.1f %.1f %.1f\n",CompFilter_Data.roll , CompFilter_Data.rollAcc , (-1*MPU6050_Data.Gyroscope_X*250.0f/32768.0f-3.80f));
		USART_WriteString(comp_sprintf_buffer);
	 	osDelay(10);
		
		//sprintf(pid_sprintf_buffer,"ch2:%.0f %d %.0f\n",20.5, PID_outputFiltered, PID1_Data.PID_output);
		sprintf(pid_sprintf_buffer,"ch2:%.0f %.0f %.0f\n",PID2_Data.PID_integral, PID2_Data.PID_output*100, PID1_Data.PID_output);
		USART_WriteString(pid_sprintf_buffer);
    osDelay(10);
  }
  /* USER CODE END DataSendLVFunction */
}

/* AdcTask function */
void AdcTask(void const * argument)
{
  /* USER CODE BEGIN AdcTask */
	
	 static char batt_sprintf_buffer[32];
	 float ADC_Vref=2.9155; 								//max voltage by divider 13,498 - 4,630 razy wieksze (4.476)
	 float ADC_VrefBattery=ADC_Vref*4.629f;
	 float ADC_VBattery=0;
   static float ADC_Resolution = 4096;
	
  /* Infinite loop */
  for(;;)
  {
		HAL_ADC_Start_DMA(&hadc1, &ValueADC1, 1); 								//ADC Measurement
	  osDelay(100);																							//Wait to done reading
		ADC_VBattery=(ValueADC1*ADC_VrefBattery)/ADC_Resolution;
		sprintf(batt_sprintf_buffer,"bat:%.2f\n",ADC_VBattery);
		USART_WriteString(batt_sprintf_buffer);
    osDelay(3900);
  }
  /* USER CODE END AdcTask */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
