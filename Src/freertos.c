/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
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
#include "usart.h"

#include "framework_debug.h"
#include "framework_iopool.h"
#include "framework_led.h"
#include "framework_remotecontrol.h"
#include "framework_motorcan.h"
#include "framework_mpu6050.h"
#include "framework_uart.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;

/* USER CODE BEGIN Variables */
osThreadId ledGTaskHandle;
osThreadId ledRTaskHandle;
osThreadId printRcTaskHandle;
osThreadId printMotorTaskHandle;
osThreadId controlMotorTaskTaskHandle;
osThreadId motorCanTransmitTaskHandle;
osThreadId printMPU6050TaskHandle;
osThreadId readMPU6050TaskHandle;
osThreadId printCtrlUartTaskHandle;
//osMessageQId uart1MessageHandle;

osSemaphoreId motorCanTransmitSemaphoreHandle;
osSemaphoreId readMPU6050SemaphoreHandle;
osSemaphoreId refreshMPU6050SemaphoreHandle;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	//wait for devices
	for(int i=0; i < 3000; i++)
	{
		int a=42000; //at 168MHz 42000 is ok
		while(a--);
	}
	
	rcInit();
	ctrlUartInit();
	motorInit();
	mpu6050Init();
	Init_Quaternion();
	fw_printfln("init success");
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
	osSemaphoreDef(motorCanTransmitSemaphore);
	motorCanTransmitSemaphoreHandle = osSemaphoreCreate(osSemaphore(motorCanTransmitSemaphore), 1);
	osSemaphoreDef(readMPU6050Semaphore);
	readMPU6050SemaphoreHandle = osSemaphoreCreate(osSemaphore(readMPU6050Semaphore), 1);
	osSemaphoreDef(refreshMPU6050Semaphore);
	refreshMPU6050SemaphoreHandle = osSemaphoreCreate(osSemaphore(refreshMPU6050Semaphore), 1);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	osThreadDef(ledGTask, ledGTask, osPriorityNormal, 0, 128);
  ledGTaskHandle = osThreadCreate(osThread(ledGTask), NULL);
	osThreadDef(ledRTask, ledRTask, osPriorityNormal, 0, 128);
  ledRTaskHandle = osThreadCreate(osThread(ledRTask), NULL);
	
	osThreadDef(printRcTask, printRcTask, osPriorityNormal, 0, 128);
  printRcTaskHandle = osThreadCreate(osThread(printRcTask), NULL);
	
	osThreadDef(printMotorTask, printMotorTask, osPriorityNormal, 0, 128);
  printMotorTaskHandle = osThreadCreate(osThread(printMotorTask), NULL);
	osThreadDef(controlMotorTask, controlMotorTask, osPriorityNormal, 0, 128);
  controlMotorTaskTaskHandle = osThreadCreate(osThread(controlMotorTask), NULL);
	osThreadDef(motorCanTransmitTask, motorCanTransmitTask, osPriorityNormal, 0, 128);
  motorCanTransmitTaskHandle = osThreadCreate(osThread(motorCanTransmitTask), NULL);
	
	osThreadDef(printMPU6050Task, printMPU6050Task, osPriorityNormal, 0, 128);
  printMPU6050TaskHandle = osThreadCreate(osThread(printMPU6050Task), NULL);
	osThreadDef(readMPU6050Task, readMPU6050Task, osPriorityNormal, 0, 128);
  readMPU6050TaskHandle = osThreadCreate(osThread(readMPU6050Task), NULL);
	
	osThreadDef(printCtrlUartTask, printCtrlUartTask, osPriorityNormal, 0, 128);
  printCtrlUartTaskHandle = osThreadCreate(osThread(printCtrlUartTask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	
//	osMessageQDef(uart1Message, 1, NULL);
//	uart1MessageHandle = osMessageCreate(osMessageQ(uart1Message), printUart1TaskHandle);

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

/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
