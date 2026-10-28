/**
 * @file    pid.c
 * @brief   通用 PID 控制器实现
 *
 * 算法说明（位置式 PID）：
 *   output = kp*err + ki*∫err + kd*d(err)/dt
 * 工程化增强：
 *   - 积分分离：|err| 超过 IntegralThreshold 时停止积分，防止大阶跃下积分饱和；
 *   - 微分一阶低通：derivative = alpha*(err-prev) + (1-alpha)*derivative，抑制微分噪声；
 *   - 积分限幅与输出限幅，防止积分饱和与输出超限。
 *
 * 工程实例化 8 组 PID：角度/高度外环（roll/pitch/yaw/height）
 * 与角速度/垂直速度内环（rollrate/pitchrate/yawrate/heightSpeed），组成串级控制。
 */
#include "pid.h"
#include "math.h"

//清除积累量,用于停止控制时使用
/**
 * @brief 复位 PID 内部状态（停止控制时调用，清空积分/微分/输出，防止下次启动跳变）
 * @param pid 目标 PID 控制器
 */
void PID_Reset(PIDControllerType_t* pid)
{
	pid->intergral = 0;
	pid->output = 0;
	pid->prev_error = 0;
	pid->derivative = 0;
}

//PID更新函数
/**
 * @brief 更新一次 PID 计算
 * @param pid     目标 PID 控制器（内含参数与状态）
 * @param target  目标值（期望角度/高度/速度等）
 * @param current 当前反馈值（传感器测量）
 * @note  结果写入 pid->output，调用方取用
 */
void PID_Update(PIDControllerType_t* pid,float target,float current)
{
	//计算误差
	float error = target - current; 
	
	//仅在设定的误差范围做积分
	if( fabs(error) < pid->IntegralThreshold ) pid->intergral += error;
	
	//微分项,采用一阶低通滤波
	float Tmpderivative = (error - pid->prev_error);
	pid->derivative = pid->alpha * Tmpderivative + (1.0f-pid->alpha) * pid->derivative;
	pid->prev_error = error;
	
	//积分限幅
	if( pid->intergral>pid->LimitIntegralMax ) pid->intergral = pid->LimitIntegralMax;
	if( pid->intergral<pid->LimitIntegralMin ) pid->intergral = pid->LimitIntegralMin;
	
	//pid输出
	pid->output = pid->kp * error + pid->ki * pid->intergral + pid->kd*pid->derivative;

	//输出限幅
	if( pid->output > pid->LimitOutputMax )  pid->output = pid->LimitOutputMax;
	if( pid->output < pid->LimitOutputMin )  pid->output = pid->LimitOutputMin;

}

//======== 8 组 PID 控制器实例（串级结构） ========
//外环：rollPID/pitchPID/yawPID/heightPID（角度/高度环）
//内环：rollratePID/pitchratePID/yawratePID/heightSpeedPID（角速度/垂直速度环）
PIDControllerType_t rollPID = {
	.kp = 4.5f,
	.ki = 0,
	.kd = 0,
	.prev_error = 0,
	.derivative = 0 ,
	.output = 0,
	.LimitIntegralMax = 5,
	.LimitIntegralMin = -5,
	.LimitOutputMax = 10,
	.LimitOutputMin = -10,
	.IntegralThreshold = 999,
	.alpha = 1
};

PIDControllerType_t rollratePID = {
	.kp = 75,
	.ki = 0.3f,
	.kd = 0,
	.prev_error = 0,
	.derivative = 0 ,
	.output = 0,
	.LimitIntegralMax = 250,
	.LimitIntegralMin = -250,
	.LimitOutputMax = 500,
	.LimitOutputMin = -500,
	.IntegralThreshold = 999,
	.alpha = 1
};

PIDControllerType_t pitchPID = {
	.kp = 4.5f,
	.ki = 0,
	.kd = 0,
	.prev_error = 0,
	.derivative = 0 ,
	.output = 0,
	.LimitIntegralMax = 5,
	.LimitIntegralMin = -5,
	.LimitOutputMax = 10,
	.LimitOutputMin = -10,
	.IntegralThreshold = 999,
	.alpha = 1
};

PIDControllerType_t pitchratePID = {
	.kp = 75,
	.ki = 0.3f,
	.kd = 0,
	.prev_error = 0,
	.derivative = 0 ,
	.output = 0,
	.LimitIntegralMax = 250,
	.LimitIntegralMin = -250,
	.LimitOutputMax = 500,
	.LimitOutputMin = -500,
	.IntegralThreshold = 999,
	.alpha = 1
};

PIDControllerType_t yawPID = {
	.kp = 6.0f,
	.ki = 0,
	.kd = 0,
	.prev_error = 0,
	.derivative = 0 ,
	.output = 0,
	.LimitIntegralMax = 2.5,
	.LimitIntegralMin = -2.5,
	.LimitOutputMax = 10,
	.LimitOutputMin = -10,
	.IntegralThreshold = 999,
	.alpha = 1
};

PIDControllerType_t yawratePID = {
	.kp = 180.0f,
	.ki = 1.0f,
	.kd = 0,
	.prev_error = 0,
	.derivative = 0 ,
	.output = 0,
	.LimitIntegralMax = 50,
	.LimitIntegralMin = -50,
	.LimitOutputMax = 400,
	.LimitOutputMin = -400,
	.IntegralThreshold = 999,
	.alpha = 1
};

PIDControllerType_t heightSpeedPID = {
	.kp = 60.0f,
	.ki = 0.02f,
	.kd = 80.0f,
	.LimitIntegralMax = 250.0f,
	.LimitIntegralMin = -250.0f,
	.LimitOutputMax = 500,
	.LimitOutputMin = -500,
	.IntegralThreshold = 20 ,
	.alpha = 0.4,
	.prev_error = 0,
	.intergral = 0,
	.derivative = 0,
	.output = 0
};

PIDControllerType_t heightPID = {
	.kp = 3.5f,
	.ki = 0.0f,
	.kd = 0.5f,
	.LimitIntegralMax = 0.5f,
	.LimitIntegralMin = -0.5f,
	.LimitOutputMax = 1.0f,
	.LimitOutputMin = -1.0f,
	.IntegralThreshold = 10 ,
	.alpha = 1.0f,
	.prev_error = 0,
	.intergral = 0,
	.derivative = 0,
	.output = 0
};


