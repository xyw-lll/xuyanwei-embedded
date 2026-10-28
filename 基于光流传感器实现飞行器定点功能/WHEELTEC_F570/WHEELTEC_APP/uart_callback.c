/**
 * @file    uart_callback.c
 * @brief   串口中断回调：数据分发入口
 *
 * 负责所有串口的接收中断处理：
 *   - UART1：预留调试串口（含软件复位指令检测）；
 *   - UART4：蓝牙串口，逐字节送入蓝牙队列供调参任务解析；
 *   - UART5：STP23L 激光雷达 DMA 接收，整包送入原始队列；
 *   - UART3：LC307 光流传感器 DMA 接收，整包送入原始队列。
 * 所有回调均为中断上下文，使用 FromISR 版本队列接口并主动触发调度。
 */
#include "usart.h"

//C Lib Include File
#include <stdio.h>

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "queue.h"

//HAL Include File

//BSP Include File
#include "bsp_stp23L.h"
#include "bsp_LC307.h"

extern uint8_t uart1_recv;
extern uint8_t uart4_recv;

//软件复位进BootLoader
/**
 * @brief 软件复位指令检测：收到连续字符 "reset" 后执行软件复位进 BootLoader
 * @param uart_recv 当前串口接收到的字节
 */
static void _System_Reset_(uint8_t uart_recv)
{
	extern void ResetSystem(uint8_t isFromISR);
	
	static uint8_t res_buf[5];
	static uint8_t res_count=0;
	
	res_buf[res_count]=uart_recv;
	
	if( uart_recv=='r'||res_count>0 )
		res_count++;
	else
		res_count = 0;
	
	if(res_count==5)
	{
		res_count = 0;
		//接受到上位机请求的复位字符“reset”，执行软件复位
		if( res_buf[0]=='r'&&res_buf[1]=='e'&&res_buf[2]=='s'&&res_buf[3]=='e'&&res_buf[4]=='t' )
		{
			NVIC_SystemReset();//进行软件复位，复位后执行 BootLoader 程序
		}
	}
}


//串口接收中断回调函数
/**
 * @brief 串口单字节接收完成中断回调（蓝牙/调试串口）
 * @param huart 触发中断的串口句柄
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//检查FreeRTOS任务是否有高优先级唤醒
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	if( huart == &huart4 ) //蓝牙使用的串口
	{
		extern QueueHandle_t g_xQueueBlueTooth;
		
		//复位BootLoadr检测指令
		_System_Reset_(uart4_recv);
		
		//把接收到的字符写入队列
		xQueueSendFromISR(g_xQueueBlueTooth,&uart4_recv,&xHigherPriorityTaskWoken);
		
		HAL_UART_Receive_IT(&huart4,&uart4_recv,1);
	}
	else if( huart == &huart1 )
	{
		_System_Reset_(uart1_recv);
		HAL_UART_Receive_IT(&huart1,&uart1_recv,1);
	}
	
	//根据具体情况发起调度
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


//DMA传输完成与半完成中断入口函数
/**
 * @brief UART DMA 空闲中断回调（激光/光流整包接收）
 * @param huart 触发中断的串口句柄
 * @param Size  DMA 已搬运的数据字节数
 * @note  将整包原始数据拷贝入队列后重新启动 DMA 接收
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	//检查高优先级任务唤醒
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if( huart == &huart5 && HAL_UART_STATE_READY == huart->RxState ) 
	{	
		//Size存放的内容是DMA已经搬运的数据数量
		
		extern QueueHandle_t g_xQueuestp23L_Ori;
		if( g_xQueuestp23L_Ori!=NULL )
		{
			DMABuf_oridata_stp23L.DataLen = Size;
			xQueueSendFromISR(g_xQueuestp23L_Ori,&DMABuf_oridata_stp23L,&xHigherPriorityTaskWoken);
		}
		
		//直接在中断里解析处理数据，有可能比较耗时，影响其他任务调度的实时性

		//重新启动DMA搬运
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5,DMABuf_oridata_stp23L.Buf,userconfig_STP23L_DMABUF_LEN);
	}
	else if( huart == &huart3 && HAL_UART_STATE_READY == huart->RxState )
	{
		extern QueueHandle_t g_xQueueLC307_Ori;
		if( g_xQueueLC307_Ori!=NULL )
		{
			DMABuf_oridata_LC307.DataLen = Size;
			xQueueSendFromISR(g_xQueueLC307_Ori,&DMABuf_oridata_LC307,&xHigherPriorityTaskWoken);
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3,DMABuf_oridata_LC307.buffer,userconfig_LC307_DMABUF_LEN);
	}
	
	
	//主动发起调度
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

