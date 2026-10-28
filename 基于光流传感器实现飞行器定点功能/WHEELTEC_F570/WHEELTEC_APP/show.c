/**
 * @file    show.c
 * @brief   调试数据上报任务
 *
 * 以 20Hz（50ms 周期）读取全局调试结构体 debugshow 与电流采样，
 * 按固定协议帧 {B...}$ 格式化后通过 UART4 DMA 发送给上位机，
 * 用于上位机实时显示姿态、目标值与高度等飞行状态。
 */
//C Lib Include File
#include <stdio.h>
#include <string.h>

//HAL Include File
#include "usart.h"

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"

//BSP Include File
#include "bsp_imu.h"
#include "bsp_adc.h"

//APP Include File
#include "balance.h"

static char showbuffer[256];

/**
 * @brief 调试数据上报任务：周期性打包并发送飞行状态
 * @param param 任务参数（未使用）
 * @note  帧格式 {B:pitch:targetPitch:roll:targetRoll:yaw:targetYaw:height:heightSpeed:0:0}$
 *        电流换算：ADC(0~4095) -> 3.3V -> /0.2V/A
 */
void Show_Task(void *param)
{
	pADCInterface_t adc = &UserADC1;
	
	while(1)
	{
		float cur = adc->getValue(userconfigADC_Curr_CHANNEL);
		cur = ((cur/4095.0f) * 3.3f)/0.2f;
		
		sprintf(showbuffer,"{B%.3f:%.3f:%.3f:%.3f:%.3f:%.3f:%.3f:%.3f:%.3f:%.3f}$",
		debugshow.pitch,
		debugshow.targetPitch,
		debugshow.roll,
		debugshow.targetRoll,
		debugshow.yaw,
		debugshow.targetYaw,
		debugshow.height,
		debugshow.heightSpeed,
		0.0f,
		0.0f
		); 
		
		//通过DMA传输数据
		HAL_UART_Transmit_DMA(&huart4,(uint8_t*)showbuffer,strlen(showbuffer));
		
		vTaskDelay( pdMS_TO_TICKS(50) );
	}
	
}



