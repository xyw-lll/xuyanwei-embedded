/**
 * @file    LC307_task.c
 * @brief   光流传感器数据处理任务（水平定点核心数据源）
 *
 * 从 DMA 队列接收 LC307 光流原始字节流，按帧头 0xFE 0x0A 进行
 * 状态机组帧，通过 XOR 校验后解析出光流速度，换算并与机体坐标
 * 对齐后写入位置队列 g_xQueueFlyPosMsg，供飞控任务融合使用。
 */
#include "bsp_LC307.h"

//C Lib Include File
#include <stdio.h>
#include <string.h>

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "queue.h"

//BSP Include File
#include "bsp_RTOSdebug.h"

//App Include File
#include "balance.h"

//接收队列数据
static OriData_LC307_t queue_recv;

//光流传感器数据帧与数据长度
static OpticalFlowFrame_t lc307_sensor;
static const uint16_t bufferLen = sizeof(OpticalFlowFrame_t);
static uint8_t lc307_buffer[bufferLen];
static uint8_t bufferCnt = 0;

//传感器调试
//static pRtosDebugInterface_t debug = &RTOSTaskDebug;
//static RtosDebugPrivateVar priv = { 0 };

/**
 * @brief 光流数据解析任务：阻塞读取原始数据队列并解析光流帧
 * @param param 任务参数（未使用）
 * @note  帧结构见 OpticalFlowFrame_t；flowX/flowY 为原始光流速度，
 *        此处取反与机体坐标对齐（与传感器安装方向有关）。
 */
void LC307_Task(void *param)
{
	extern QueueHandle_t g_xQueueLC307_Ori;
	extern QueueHandle_t g_xQueueFlyPosMsg;
	
	//光流数据解析辅助变量
	static uint8_t state_m = 0;
	static uint8_t LastRecv = 0;
	
	FlyPosMsg_t pos;
	
	while(1)
	{
		//读取光流传感器数据
		xQueueReceive(g_xQueueLC307_Ori,&queue_recv,portMAX_DELAY);
		
		//解析数据
		for(uint8_t i=0;i<queue_recv.DataLen;i++)
		{
			switch( state_m )
			{
				case 0:
					if( queue_recv.buffer[i]==0x0A && LastRecv == 0xFE )
					{
						lc307_buffer[0] = 0xFE;
						lc307_buffer[1] = 0x0A;
						bufferCnt=2;
						state_m=1;
					}
					break;
				case 1:
					lc307_buffer[bufferCnt++] = queue_recv.buffer[i];
					if( bufferCnt == bufferLen )
					{
						//解析数据
						//整帧接收完成，校验倒数第 2 字节（XOR 校验和），通过后解析
						if( lc307_buffer[bufferLen-2] == XOR_Checksum(&lc307_buffer[2],10) )
						{
							memcpy(&lc307_sensor,lc307_buffer,bufferLen);
							
							//将光流速度写入位置队列（x/y 与机体坐标对齐）
							//写队列
							pos.x_speed = -lc307_sensor.flowY; //与飞行器坐标对齐(跟安装的位置和方向有关系)
							pos.y_speed = -lc307_sensor.flowX;
							xQueueSend(g_xQueueFlyPosMsg,&pos,0);
							
							//调试
//							printf( "LC307 Freq:%d\r\n",debug->UpdateFreq(&priv));
						}

						//状态机复位
						state_m=0;
					}
					break;
				default:
					break;
			}
			
			LastRecv = queue_recv.buffer[i];
		}

	}
}
