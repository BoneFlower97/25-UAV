/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "global.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
SemaphoreHandle_t  sensorsDataReady;
SemaphoreHandle_t  icmDataReady;
//SemaphoreHandle_t uart1Mutex;
SemaphoreHandle_t uart1TxDone;

xQueueHandle accelerometerDataQueue;
xQueueHandle barometerDataQueue;
xQueueHandle gyroDataQueue;
xQueueHandle stateDataQueue;
xQueueHandle sensorDataQueue;
xQueueHandle uartRxQueue;
xQueueHandle controlcommandDataQueue;
xQueueHandle rcctrlDataQueue;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId START_TASKHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void startTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  sensorsDataReady = xSemaphoreCreateBinary();/*创建传感器数�?就绪二值信号量*/
  icmDataReady = xSemaphoreCreateBinary();
  //uart1Mutex = xSemaphoreCreateMutex();
  uart1TxDone = xSemaphoreCreateBinary();

	barometerDataQueue = xQueueCreate(1, sizeof(baro_t));
  accelerometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
	gyroDataQueue = xQueueCreate(1, sizeof(Axis3f));
	sensorDataQueue = xQueueCreate(1, sizeof(sensorData_t));
	stateDataQueue = xQueueCreate(1, sizeof(state_t));
  uartRxQueue = xQueueCreate(1, sizeof(uart_frame_t));
  controlcommandDataQueue = xQueueCreate(1, sizeof(rc_ctrl_t));
	rcctrlDataQueue = xQueueCreate(1, sizeof(control_command_t));
	
	

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of START_TASK */
  osThreadDef(START_TASK, startTask, osPriorityNormal, 0, 300);
  START_TASKHandle = osThreadCreate(osThread(START_TASK), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
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

/* USER CODE BEGIN Header_startTask */
/**
* @brief Function implementing the START_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startTask */
void startTask(void const * argument)
{
  /* USER CODE BEGIN startTask */
  taskENTER_CRITICAL();	/*½øÈëÁÙ½çÇø*/
  xTaskCreate(linkTxTask, "LINKTXTASK", 150, NULL, 3, NULL);    /*´´½¨½ÓÊÕÈÎÎñ*/
  //xTaskCreate(linkRxTask, "LINKRXTASK", 150, NULL, 2, NULL);
  xTaskCreate(sensorsTask, "SENSORS", 450, NULL, 4, NULL);			/*´´½¨´«¸ÐÆ÷´¦ÀíÈÎÎñ*/
	xTaskCreate(stabilizerTask, "STABILIZER", 450, NULL, 5, NULL);		/*������̬����*/
  //xTaskCreate(wirelessTxTask, "WIRELESSTXTASK", 150, NULL, 3, NULL);				/*´´½¨atkp·¢ËÍÈÎÎñÈÎÎñ*/  
  //xTaskCreate(wirelessRxTask, "WIRELESSRXTASK", 300, NULL, 6, NULL);				/*´´½¨atkp·¢ËÍÈÎÎñÈÎÎñ*/  
	xTaskCreate(wirelessTask, "WIRELESSTASK", 450, NULL, 6, NULL);
	//xTaskCreate(staTask, "STATASK", 100, NULL, 6, NULL);
  vTaskDelete(START_TASKHandle);										/*É¾³ý¿ªÊ¼ÈÎÎñ*/

  taskEXIT_CRITICAL();	/*ÍË³öÁÙ½çÇø*/
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END startTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
