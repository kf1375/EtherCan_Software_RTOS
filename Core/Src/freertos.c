/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SOCKET_NUMBER 	0
#define UDP_PORT				56800
#define UDP_BUF_SIZE   	2048

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern void 		Ethernet_Init(void);
extern int32_t 	UDP_Loop(uint8_t sn, uint8_t* buf, uint16_t port);

uint8_t gDATABUF[UDP_BUF_SIZE];

/* USER CODE END Variables */
/* Definitions for ethernetHandler */
osThreadId_t ethernetHandlerHandle;
const osThreadAttr_t ethernetHandler_attributes = {
  .name = "ethernetHandler",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for canIRQHandler */
osThreadId_t canIRQHandlerHandle;
const osThreadAttr_t canIRQHandler_attributes = {
  .name = "canIRQHandler",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartEthernetHandlerTask(void *argument);
void StartCanIRQHandlerTask(void *argument);

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ethernetHandler */
  ethernetHandlerHandle = osThreadNew(StartEthernetHandlerTask, NULL, &ethernetHandler_attributes);

  /* creation of canIRQHandler */
  canIRQHandlerHandle = osThreadNew(StartCanIRQHandlerTask, NULL, &canIRQHandler_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartEthernetHandlerTask */
/**
  * @brief  Function implementing the ethernetHandler thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartEthernetHandlerTask */
void StartEthernetHandlerTask(void *argument)
{
  /* USER CODE BEGIN StartEthernetHandlerTask */
	Ethernet_Init();

  /* Infinite loop */
  for(;;)
  {
    UDP_Loop(SOCKET_NUMBER, gDATABUF, UDP_PORT);
    osDelay(100);
  }
  /* USER CODE END StartEthernetHandlerTask */
}

/* USER CODE BEGIN Header_StartCanIRQHandlerTask */
/**
* @brief Function implementing the canIRQHandler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanIRQHandlerTask */
void StartCanIRQHandlerTask(void *argument)
{
  /* USER CODE BEGIN StartCanIRQHandlerTask */
  /* Infinite loop */
  for(;;)
  {
  	HAL_GPIO_TogglePin(LED_TX_GPIO_Port, LED_TX_Pin);
  	osDelay(100);
  }
  /* USER CODE END StartCanIRQHandlerTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

