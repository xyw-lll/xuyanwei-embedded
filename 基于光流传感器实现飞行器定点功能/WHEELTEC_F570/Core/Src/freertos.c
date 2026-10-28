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

//C Lib Include File
#include <stdio.h>

//FreeRTOS Include File
#include "queue.h"
#include "timers.h"

//HAL Include File
#include "tim.h"
#include "usart.h"

//BSP Include File
#include "bsp_stp23L.h"
#include "bsp_buzzer.h"
#include "bsp_LC307.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define userconfig_RTOS_DEBUG 0 //是否开启调试任务
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

//定义STP23L模块的队列
QueueHandle_t g_xQueuestp23L_Ori=NULL;//存放STP23L原始数据

//定义LC307光流模块的队列
QueueHandle_t g_xQueueLC307_Ori=NULL;

//定义蓝牙数据传输的队列
QueueHandle_t g_xQueueBlueTooth = NULL;

//定义定时器
//操作成功定时器
TimerHandle_t Timer_SuccessTips;

//操作失败定时器
TimerHandle_t Timer_FailTips;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

#if ( 1 == userconfig_RTOS_DEBUG )
void RTOSDebugTask(void *param);
#endif

void Balance_Task(void *param);
void Show_Task(void *param);
void STP23L_Task(void *param);
void BlueTooth_Task(void *param);
void PS2_Task(void *param);
void LC307_Task(void *param);

//定时器回调函数
void Timer_SuccessCallback(TimerHandle_t xTimer);
void Timer_FailCallback(TimerHandle_t xTimer);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_HOST_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{
	HAL_TIM_Base_Start(&htim6);
}

__weak unsigned long getRunTimeCounterValue(void)
{
	static unsigned long time = 0 ;
	static uint16_t lasttime = 0;
	static uint16_t nowtime = 0;
	
	nowtime = TIM6->CNT; //获得当前计数值
	
	//如果本次计数值小于上次计数值，说明发生了定时器计数溢出
	if( nowtime < lasttime )
	{
		time += (nowtime + 0xffff - lasttime); //溢出后的时间增量
	}		
	else time += ( nowtime - lasttime ) ; //未发生溢出，增量为本次时间-上次时间
	
	lasttime = nowtime;
	
	return time;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
	
	//空闲任务回调函数
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
	
	//任务栈溢出
	printf("%s stack overflow\r\n",pcTaskName);
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
	printf("Malloc Faile,check total heap size config\r\n");
}
/* USER CODE END 5 */

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
	Timer_SuccessTips = xTimerCreate("SucceessTimer",pdMS_TO_TICKS(1),pdFALSE,NULL,Timer_SuccessCallback);
	Timer_FailTips = xTimerCreate("Failtimer",pdMS_TO_TICKS(1),pdFALSE,NULL,Timer_FailCallback);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	
	//创建STP23L激光测距模块队列
	g_xQueuestp23L_Ori = xQueueCreate(3,sizeof(OriData_STP23L_t));
	
	//创建光流传感器队列
	g_xQueueLC307_Ori = xQueueCreate(3,sizeof(OriData_LC307_t));
	
	//创建蓝牙数据队列
	g_xQueueBlueTooth = xQueueCreate(50,sizeof(char));
	
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	
	//平衡控制任务
	xTaskCreate(Balance_Task,"balance",128*4,NULL,osPriorityNormal,NULL);
	
	//调试输出任务
	xTaskCreate(Show_Task,"showtask",128*4,NULL,osPriorityBelowNormal7,NULL);
	
	//创建STP23L数据处理任务
	xTaskCreate(STP23L_Task,"STP23L_Task",128*2,NULL,osPriorityNormal,NULL);
	
	//创建光流传感器数据解析任务
	xTaskCreate(LC307_Task,"LC307_Task",128*2,NULL,osPriorityNormal,NULL);
	
	//创建蓝牙数据调参任务
	xTaskCreate(BlueTooth_Task,"BT_Task",128*2,NULL,osPriorityNormal,NULL);
	
	//创建USB手柄控制任务
	xTaskCreate(PS2_Task,"PS2_Task",128*2,NULL,osPriorityNormal,NULL);
	
	//创建FreeRTOS调试任务
	#if ( 1 == userconfig_RTOS_DEBUG )
	 static uint16_t delaytime = 5000;//打印时间间隔，单位tick
	 xTaskCreate(RTOSDebugTask,"DebugTask",128, &delaytime ,osPriorityAboveNormal,NULL);
	#endif
	
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	
	//启动STP23L激光测距模块的DMA搬运
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5,DMABuf_oridata_stp23L.Buf,userconfig_STP23L_DMABUF_LEN);
	
	//启动光流传感器的DMA搬运
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3,DMABuf_oridata_LC307.buffer,userconfig_LC307_DMABUF_LEN);
  for(;;)
  {
    HAL_GPIO_TogglePin(UserLED1_GPIO_Port,UserLED1_Pin);
		HAL_GPIO_TogglePin(UserLED2_GPIO_Port,UserLED2_Pin);
		//HAL_Delay(200);
		vTaskDelay( pdMS_TO_TICKS(500) );
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
#if ( 1 == userconfig_RTOS_DEBUG ) 
void RTOSDebugTask(void *param)
{
	uint16_t* delaytime = (uint16_t*)param;
	static char showbuf[500];
	
	while( 1 )
	{
		//打印CPU占比
		vTaskGetRunTimeStats(showbuf);
		printf("TaskName\tUseTime\tCPU\r\n");
		printf("%s\r\n",showbuf);
		vTaskDelay(*delaytime);

		//打印剩余任务栈大小,单位word
		vTaskList(showbuf);
		printf("TaskName\tTaskState\tTaskPrio\tStackSize\tTaskNum\r\n");
		printf("%s\r\n",showbuf);
		vTaskDelay(*delaytime);
		
		//打印剩余的堆区大小,单位bytes
		printf("free heap size : %d bytes\r\n\r\n",xPortGetFreeHeapSize());
		vTaskDelay(*delaytime);
	}
}
#endif


void Timer_SuccessCallback(TimerHandle_t xTimer)
{
	pBuzzeInterface_t tips = &UserBuzzer;
	tips->on();
	vTaskDelay(50);
	tips->off();
}

void Timer_FailCallback(TimerHandle_t xTimer)
{
	pBuzzeInterface_t tips = &UserBuzzer;
	for(uint8_t i=0;i<3;i++)
	{
		tips->on();
		vTaskDelay(50);
		tips->off();
		vTaskDelay(50);
	}
}


/* USER CODE END Application */

