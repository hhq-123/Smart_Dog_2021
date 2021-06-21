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
#include "stdio.h"
#include "usart.h"


#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "queue.h"
#include "semphr.h"

#include "LobotServoController.h"
#include "Quadrudep_huaner.h"
#include "test_workspace.h"

#include "Dog_interface.h"
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

uint16_t  LSCControlTimerCount = 0;
extern uint16_t LSCControlPeriod;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1024 * 4
};
/* Definitions for usart1RxTask */
osThreadId_t usart1RxTaskHandle;
const osThreadAttr_t usart1RxTask_attributes = {
  .name = "usart1RxTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for usart1RxMsgQueue */
osMessageQueueId_t usart1RxMsgQueueHandle;
const osMessageQueueAttr_t usart1RxMsgQueue_attributes = {
  .name = "usart1RxMsgQueue"
};
/* Definitions for LSCControlTimer */
osTimerId_t LSCControlTimerHandle;
const osTimerAttr_t LSCControlTimer_attributes = {
  .name = "LSCControlTimer"
};
/* Definitions for gaitControlBinarySem */
osSemaphoreId_t gaitControlBinarySemHandle;
const osSemaphoreAttr_t gaitControlBinarySem_attributes = {
  .name = "gaitControlBinarySem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartUsart1RxTask(void *argument);
void LSCControlCallback(void *argument);

extern void MX_USB_DEVICE_Init(void);
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

  /* Create the semaphores(s) */
  /* creation of gaitControlBinarySem */
  gaitControlBinarySemHandle = osSemaphoreNew(1, 1, &gaitControlBinarySem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of LSCControlTimer */
  LSCControlTimerHandle = osTimerNew(LSCControlCallback, osTimerPeriodic, NULL, &LSCControlTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of usart1RxMsgQueue */
  usart1RxMsgQueueHandle = osMessageQueueNew (3, sizeof(USART_RECEIVETYPE), &usart1RxMsgQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of usart1RxTask */
  usart1RxTaskHandle = osThreadNew(StartUsart1RxTask, NULL, &usart1RxTask_attributes);

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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	osTimerStart(LSCControlTimerHandle,1);
  for(;;)
  {
		Gait_Controller();
    //osDelay(1);
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
	
	printf("Start StartUsart1RxTask\r\n");
  /* Infinite loop */
  for(;;)
  {
		xQueueReceive(usart1RxMsgQueueHandle, (void*)&pUARTR1, osWaitForever);
		printf("UARTR1: \"%s \" ends\r\n", pUARTR1.RxBuff);
		bluetoothTranslater(pUARTR1.RxBuff);
  }
  /* USER CODE END StartUsart1RxTask */
}

/* LSCControlCallback function */
void LSCControlCallback(void *argument)
{
  /* USER CODE BEGIN LSCControlCallback */
	
	
	LSCControlTimerCount++;
	if(LSCControlTimerCount >= LSCControlPeriod)
	{
		LSC_communication();
		xSemaphoreGive(gaitControlBinarySemHandle);
		LSCControlTimerCount = 0;
	}
  /* USER CODE END LSCControlCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
