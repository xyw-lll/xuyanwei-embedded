/**
 * @file    pid.h
 * @brief   通用 PID 控制器结构体与接口声明
 *
 * 结构体同时保存 PID 三参数与运行状态（误差/积分/微分/输出），
 * 并内置积分分离阈值、积分/输出限幅与微分低通滤波系数。
 */
#ifndef __PID_H
#define __PID_H

#include <stdint.h>

//PID 控制器结构体：参数与运行状态一体，支持在线整定
typedef struct{
	float kp; // 比例增益
	float ki; // 积分增益
	float kd; // 微分增益
	float prev_error; // 上一次误差
	float intergral;  // 积分项
	float derivative; //微分项
	float output;     // 输出
	float LimitIntegralMax; //积分上限
	float LimitIntegralMin; //积分下限
	float LimitOutputMax;   //输出上限
	float LimitOutputMin;   //输出下限
	float IntegralThreshold;//积分阈值
	float alpha;            //一阶低通滤波系数
}PIDControllerType_t;

void PID_Update(PIDControllerType_t* pid,float target,float current);
void PID_Reset(PIDControllerType_t* pid);

extern PIDControllerType_t rollPID,rollratePID,pitchPID,pitchratePID,yawPID,yawratePID,heightPID,heightSpeedPID;

#endif

