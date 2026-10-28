/**
 * @file    stp23l_task.c
 * @brief   STP23L 激光测距任务（定高数据源）
 *
 * 阻塞等待 STP23L 原始数据队列（由串口 DMA 中断填充），将原始
 * 帧交给 stp23L_callback() 解析，得到对地高度 g_readonly_distance，
 * 供飞控任务计算当前高度与垂直速度。
 */
#include "bsp_stp23L.h"

//C Lib Include File
#include <stdio.h>

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "queue.h"

static OriData_STP23L_t queue_recv;

/**
 * @brief 激光测距任务：读取原始数据队列并解析高度
 * @param param 任务参数（未使用）
 */
void STP23L_Task(void *param)
{
	extern QueueHandle_t g_xQueuestp23L_Ori;
	
	while(1)
	{
		//读取STP23L数据队列
		xQueueReceive(g_xQueuestp23L_Ori,&queue_recv, portMAX_DELAY ); //等待队列数据
		
		//解析STP23L原始数据,得到高度数据
		stp23L_callback(&queue_recv);
		
	}
}


