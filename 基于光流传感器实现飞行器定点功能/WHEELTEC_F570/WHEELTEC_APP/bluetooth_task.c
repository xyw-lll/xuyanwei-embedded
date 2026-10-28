/**
 * @file    bluetooth_task.c
 * @brief   蓝牙串口数据任务：在线 PID 参数整定
 *
 * 通过蓝牙接收上位机下发的 PID 参数帧，格式为：{S0:kp,ki,kd}
 * （S 后的数字 0~7 表示 8 组 PID 的编号），解析成功后写入对应的
 * PID 控制器参数并蜂鸣提示。用于不重新烧录即可在线调参。
 */
//C Inlucde File
#include <stdio.h>

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

//BSP Include File

//App Include File
#include "pid.h"
#include "balance.h"

extern TimerHandle_t Timer_SuccessTips;
extern TimerHandle_t Timer_FailTips;

/**
 * @brief 蓝牙 PID 调参任务：阻塞等待蓝牙队列字节并解析参数帧
 * @param param 任务参数（未使用）
 * @note  帧格式：{S<组号0~7>:<kp>,<ki>,<kd>}，组号对应 8 组串级 PID 控制器
 */
void BlueTooth_Task(void *param)
{
	extern QueueHandle_t g_xQueueBlueTooth;
	
	//接收蓝牙队列数据
	uint8_t bluetooth_recv=0;
	
	//上一次数据内容
	uint8_t last_recv = 0;
	
	//用于PID参数组接收
	#define pidparam_len 50
	char pid_param[pidparam_len];
	uint8_t pid_cnt = 0;
	uint8_t state_sm=0;
	uint8_t ready = 0;
	
	//用于pid参数解析
	uint8_t param_type;
	float kp,ki,kd;
	
	//pid参数格式示例：{S0:1.0,-2.1,15}  //设置数据组0,kp=1.0，ki=-2.1，kd=15
	
	while(1)
	{
		xQueueReceive(g_xQueueBlueTooth,&bluetooth_recv, portMAX_DELAY ); //等待队列数据
		
		//PID 参数帧解析状态机：
		//state_sm=0 等待帧头 '{'+'S'；state_sm=1 接收完整参数串直到 '}'
		//PID参数接收
		switch( state_sm )
		{
			case 0:
				if( last_recv=='{' && bluetooth_recv=='S' )
				{
					pid_param[0]='{';
					pid_param[1]='S';
					pid_cnt=2;
					state_sm=1;
				}
				break;
			case 1:
				//接收pid字符数据
				pid_param[pid_cnt++] = bluetooth_recv;
				if( pid_cnt >= pidparam_len-2 )
				{
					//超出缓冲区,弃用本次数据
					pid_cnt=0;
					state_sm=0;
				}
				else if( bluetooth_recv=='}' )
				{
					//开始解析
					pid_param[pid_cnt]='\0';//加上结束符
					ready=1;
					state_sm=0; //重新开始接收
					pid_cnt=0;
				}
				break;
			default:
				break;
		}
	
		//解析成功后将 kp/ki/kd 写入对应 PID（0~7 号），并触发成功提示音
		//数据赋值部分
		if( ready )
		{
			ready=0;
			uint8_t res = sscanf(pid_param,"{S%c:%f,%f,%f}",&param_type,&kp,&ki,&kd);
			
			if( res == 4 ) //成功解析
			{
				//蜂鸣器提示参数被设置
				xTimerStart(Timer_SuccessTips,0);
				
				param_type = param_type-'0';
				
				//调试输出
//				printf("type:%d\r\n",param_type);
//				printf("kp:%f\r\nki:%f\r\nkd:%f\r\n",kp,ki,kd);
				
				switch(param_type)
				{
					//PID 组号对应关系（串级结构）：
					//0~2: roll/pitch/yaw 角速度内环；3~5: 角度外环；6~7: 高度速度内环/高度外环
					//roll-rate内环
					case 0:
						rollratePID.kp = kp; rollratePID.ki = ki; rollratePID.kd = kd;
						break;
					
					//roll外环
					case 1:
						rollPID.kp = kp; rollPID.ki = ki; rollPID.kd = kd;
						break;
					
					//pitch-rate内环
					case 2:
						pitchratePID.kp = kp; pitchratePID.ki = ki; pitchratePID.kd = kd;
						break;
					
					//pitch外环
					case 3:
						pitchPID.kp = kp; pitchPID.ki = ki; pitchPID.kd = kd;
						break;
					
					//yaw-rate内环
					case 4:
						yawratePID.kp = kp; yawratePID.ki = ki; yawratePID.kd = kd;
						break;
					
					//yaw外环
					case 5:
						yawPID.kp = kp; yawPID.ki = ki; yawPID.kd = kd;
						break;
					
					//height-rate内环
					case 6:
						heightSpeedPID.kp = kp; heightSpeedPID.ki = ki; heightSpeedPID.kd = kd;
						break;
					
					//height外环
					case 7:
						heightPID.kp = kp; heightPID.ki = ki; heightPID.kd = kd;
						break;
					default:
						break;
					
				}
				
			}
		}

		
		last_recv = bluetooth_recv;
	}

}

