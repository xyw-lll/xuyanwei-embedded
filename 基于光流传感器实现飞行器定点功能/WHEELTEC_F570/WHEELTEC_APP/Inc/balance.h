/**
 * @file    balance.h
 * @brief   飞控核心模块公共定义
 *
 * 包含：控制指令优先级枚举、控制指令结构体、位置/速度队列结构体、
 *       调试监控结构体，以及控制队列写入接口声明。
 */
#ifndef __BALANCE
#define __BALANCE

//C Lib Inclue Flie
#include <stdint.h>

//FreeRTOS include File
#include "FreeRTOS.h"

#define PI 3.141592653f

//控制指令优先级,数值越小优先级越高
enum{
	NONE_CMD = 0, //未指定的控制来源
	
	PS2_CMD = 1, //最高优先级
	APP_CMD, //2

	UNKNOWN_CMD,  //未知的控制来源
};

//控制队列结构体
typedef struct{
	uint8_t cmdsource;//控制指令来源
	float pitch;     
	float roll;
	float yaw;
	float height;
}FlyControlCMDType_t;


//飞行器坐标数据
typedef struct{
	float x_pos;
	float y_pos;
	float x_speed;
	float y_speed;
}FlyPosMsg_t;

typedef struct{
	float pitch;
	float roll;     
	float yaw;
	float gyrox;
	float gyroy;
	float gyroz;
	float height;
	float heightSpeed;
	float targetPitch;
	float targetRoll;
	float targetYaw;
	float targetHeight;
}DebugShowType_t;

//对外调试监控数据
extern DebugShowType_t debugshow;

//控制指令写入接口：所有控制源（手柄/蓝牙等）统一经此接口下发控制量
//cmd:  控制指令（含来源与 4 通道控制量）；woken: 中断中调用时传唤醒标志地址，否则传 0
//返回 0 成功 / 1 失败（来源非法或被更高优先级抢占）
uint8_t WriteFlyControlQueue(FlyControlCMDType_t* cmd,BaseType_t* woken);


#endif
