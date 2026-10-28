/**
 * @file    ps2_task.c
 * @brief   PS2 手柄控制任务
 *
 * 以 20Hz 周期读取无线 PS2 手柄摇杆/扳机状态，将摇杆量映射为
 * 飞行器控制指令（pitch/roll/yaw/height），通过 WriteFlyControlQueue()
 * 写入统一控制队列。摇杆位于中位时指令空闲，空闲超过 1s 停止下发，
 * 使飞控进入失控保护。
 */
//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"

//BSP Include File
#include "bsp_gamepad.h"

//APP Include File
#include "balance.h"

//输入一个角度,返回对应的弧度值
/**
 * @brief 角度转弧度
 * @param angle 角度值（°）
 * @retval 对应的弧度值
 */
float angle_to_rad(float angle)
{
	return angle*0.017453292f;
}

/**
 * @brief PS2 手柄控制任务：20Hz 周期采集摇杆并下发控制指令
 * @param param 任务参数（未使用）
 * @note  摇杆量程 0~255，中位 127；<64 或 >191 视为有效输入。
 *        roll/pitch 步进 10°，yaw 1s 转 90°，height 0.5m/s。
 */
void PS2_Task(void *param)
{
	//获取时基,用于辅助任务能指定固定频率运行
	TickType_t preTime = xTaskGetTickCount();
	
	//控制任务的频率,单位HZ
	const uint16_t TaskFreq = 20;
	
	//创建控制量并指定控制来源
	FlyControlCMDType_t ps2Control = {
		.cmdsource = PS2_CMD,
		.pitch = 0,
		.roll = 0,
		.yaw = 0,
		.height = 0
	};
	
	uint8_t IdleCnt = TaskFreq;//指令空闲计数
	
	while(1)
	{
		uint8_t Idle = 0; //指令空闲判断
		
		const float step = angle_to_rad( 10 ); //固定控制值10度
		
		//roll方向
		if( GamePadInterface->LY < 64 )
		{
			//摇杆往前
			ps2Control.roll = -step;
		}
		else if( GamePadInterface->LY > 191 )
		{
			//摇杆往后
			ps2Control.roll = step;
		}
		else
		{
			Idle++;
			ps2Control.roll = 0;
		}
		
		//pitch
		if( GamePadInterface->LX < 64 )
		{
			//摇杆往左
			ps2Control.pitch = -step;
		}
		else if( GamePadInterface->LX > 191 )
		{
			//摇杆往右
			ps2Control.pitch = step;
		}
		else
		{
			Idle++;
			ps2Control.pitch = 0;
		}
		
		//yaw 1秒旋转90度
		const float yawstep = angle_to_rad( 90.0f/(float)TaskFreq );

		if( GamePadInterface->RX < 64 )
		{
			//摇杆往左(逆时针旋转)
			ps2Control.yaw = yawstep;
		}
		else if( GamePadInterface->RX > 191 )
		{
			//摇杆往右(顺时针旋转)
			ps2Control.yaw = -yawstep;
		}
		else
		{
			Idle++;
			ps2Control.yaw = 0;
		}
		
		//height
		const float heightStep = 0.5f/(float)TaskFreq;
		if( GamePadInterface->RY < 64 )
		{
			//摇杆往前
			ps2Control.height = heightStep;
		}
		else if( GamePadInterface->RY > 191 )
		{
			//摇杆往后
			ps2Control.height = -heightStep;
		}
		else
		{
			Idle++;
			ps2Control.height = 0;
		}
		
		if( Idle == 4 ) IdleCnt++;
		else IdleCnt = 0;

		//空闲超过1秒后不再写队列
		if( IdleCnt > TaskFreq*1 )
		{
			IdleCnt = TaskFreq*1 + 1;
		}
		else
		{
			WriteFlyControlQueue(&ps2Control,0);
		}
		
		/* 延迟指定频率 */
		vTaskDelayUntil(&preTime,pdMS_TO_TICKS( (1.0f/(float)TaskFreq)*1000) );
	}
}

