/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "queue.h"

#include "stdio.h"
#include "usart.h"
#include "LobotServoController.h"
#include "test_workspace.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
int isFreeRTOSSysOn = 0;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for usart1RxTask */
osThreadId_t usart1RxTaskHandle;
const osThreadAttr_t usart1RxTask_attributes = {
  .name = "usart1RxTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for blueToothRxTask */
osThreadId_t blueToothRxTaskHandle;
const osThreadAttr_t blueToothRxTask_attributes = {
  .name = "blueToothRxTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for usart1RxMsgQueue */
osMessageQueueId_t usart1RxMsgQueueHandle;
const osMessageQueueAttr_t usart1RxMsgQueue_attributes = {
  .name = "usart1RxMsgQueue"
};
/* Definitions for blueToothRxMsgQueue */
osMessageQueueId_t blueToothRxMsgQueueHandle;
const osMessageQueueAttr_t blueToothRxMsgQueue_attributes = {
  .name = "blueToothRxMsgQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartUsart1RxTask(void *argument);
void StartBlueToothRxTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
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

  /* Create the queue(s) */
  /* creation of usart1RxMsgQueue */
  usart1RxMsgQueueHandle = osMessageQueueNew (3, sizeof(USART_RECEIVETYPE), &usart1RxMsgQueue_attributes);

  /* creation of blueToothRxMsgQueue */
  blueToothRxMsgQueueHandle = osMessageQueueNew (3, sizeof(USART_RECEIVETYPE), &blueToothRxMsgQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of usart1RxTask */
  usart1RxTaskHandle = osThreadNew(StartUsart1RxTask, NULL, &usart1RxTask_attributes);

  /* creation of blueToothRxTask */
  blueToothRxTaskHandle = osThreadNew(StartBlueToothRxTask, NULL, &blueToothRxTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	isFreeRTOSSysOn = 1;
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		printf("StartDefaultTask\n\r");
		Trot_run();
		//moveServo(25, step, 50); //1秒移动1号舵机至2000位置
		//if(step++==1500)
			//step = 500;
		printf("StartDefaultTask\n\r");
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartUsart1RxTask */
/**
* @brief Function implementing the usart1RxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsart1RxTask */
void StartUsart1RxTask(void *argument)
{
  /* USER CODE BEGIN StartUsart1RxTask */
	USART_RECEIVETYPE pUARTR1;
	
	printf("3 Start usartRxFunc\r\n");
  /* Infinite loop */
  for(;;)
  {
		xQueueReceive(usart1RxMsgQueueHandle, (void*)&pUARTR1, osWaitForever);
		printf("Run\r\n");
		printf("3 Run ：%s\r\n", pUARTR1.RxBuff);
		printf("ends\r\n");
  }
  /* USER CODE END StartUsart1RxTask */
}

/* USER CODE BEGIN Header_StartBlueToothRxTask */
/**
* @brief Function implementing the blueToothRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlueToothRxTask */
void StartBlueToothRxTask(void *argument)
{
  /* USER CODE BEGIN StartBlueToothRxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartBlueToothRxTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
