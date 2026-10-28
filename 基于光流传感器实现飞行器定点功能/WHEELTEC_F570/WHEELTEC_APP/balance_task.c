/**
 * @file    balance_task.c
 * @brief   飞控核心任务：姿态稳定、定高与水平定点控制
 *
 * 模块职责：
 *   - 以 200Hz 固定频率运行飞控主循环，完成 9 轴 IMU、电压、高度、光流数据采集与状态估计；
 *   - 光流 + IMU 加速度互补滤波融合，解算水平速度与位移（水平定点的基础）；
 *   - 串级 PID 控制：位置环 -> 角度环 -> 角速度环，高度环 -> 垂直速度环；
 *   - X 型四旋翼电机混控，经 DShot300 输出 4 路油门；
 *   - 提供 WriteFlyControlQueue() 作为所有控制源的统一入口（含优先级仲裁）；
 *   - 内置开机零点标定、指令超时失控保护、倾角超限/低高度自动停桨等安全机制。
 */
#include "balance.h"

//C Lib Include File
#include <math.h>

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

//BSP Include File
#include "bsp_imu.h"
#include "bsp_RTOSdebug.h"
#include "bsp_dshot.h"
#include "bsp_gamepad.h"
#include "WiredlessPS2_gamepad.h"
#include "bsp_adc.h"
#include "bsp_buzzer.h"
#include "bsp_stp23L.h"

//APP Include File
#include "pid.h"

//提示定时器
extern TimerHandle_t Timer_SuccessTips;
extern TimerHandle_t Timer_FailTips;
//私有定时器_零点标定完成
static TimerHandle_t priv_BuzzerTipsTimer;
static void BuzzerTipsTimer_Callback(TimerHandle_t xTimer);

//私有队列,存放控制命令
static QueueHandle_t g_xQueueFlyControl = NULL;

//私有变量,表示当前的控制来源
static uint8_t FlyControl_CMDsource = NONE_CMD;

//电压获取相关变量
static pADCInterface_t adc = &UserADC1;
float robot_vol;

//IMU相关变量
static pIMUInterface_t imu = &UserICM20948;
static ATTITUDE_DATA_t attitude = { 0 };//角度值
static IMU_DATA_t imu_ori = { 0 };      //角速度的值
//IMU零点数据
static ATTITUDE_DATA_t zero_attitude = { 0 };
static IMU_DATA_t zero_imu_ori = { 0 };


//电机的油门值相关变量
static pMotorInterface_t dshot = &UserDshotMotor;
static dshotMotorVal_t mA={0,48},mB={0,48},mC={0,48},mD={0,48};

//freertos调试接口相关变量
//static pRtosDebugInterface_t debug = &RTOSTaskDebug;
//static RtosDebugPrivateVar priv = { 0 };
//static uint16_t balance_task_freq = 0;
//static float get_imu_data_time = 0;

//系统时间
static uint32_t SystemRunTime = 0;

//用于观测飞行器的状态调试变量
DebugShowType_t debugshow;

//飞行器启动标志位
static uint8_t StartFlag = 0;

//飞行器默认高度
#define DefalutHeight 1.0f
#define MaxHeight 2.0f
#define MinHeight -0.5f

//对外使用队列,输入飞行器的坐标队列
QueueHandle_t g_xQueueFlyPosMsg = NULL;

//USB手柄按键回调函数
/**
 * @brief 无线 PS2 手柄按键事件回调（由 USB 手柄驱动在任务环境中调用）
 * @param keyid 按键编号（如 PS2KEY_START）
 * @param event 按键事件类型（单击/双击/长按等）
 * @note  START 长按：切换飞控启动/停止；START 其他事件：强制停机。
 */
void WiredlessPS2_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
		xTimerStart(Timer_SuccessTips,0);
	
		//测量不同电压下的推力值方式
//	if( keyid == PS2KEY_1GREEN )
//	{
//		mA.throttle += 50;
//		mB.throttle += 50;
//		mC.throttle += 50;
//		mD.throttle += 50;
//		if( mA.throttle >= 2047 ) mA.throttle=2047;
//		if( mB.throttle >= 2047 ) mB.throttle=2047;
//		if( mC.throttle >= 2047 ) mC.throttle=2047;
//		if( mD.throttle >= 2047 ) mD.throttle=2047;
//	}
//	
//	if( keyid == PS2KEY_3BLUE )
//	{
//		mA.throttle -= 50;
//		mB.throttle -= 50;
//		mC.throttle -= 50;
//		mD.throttle -= 50;
//		
//		if( mA.throttle <= 48 ) mA.throttle=48;
//		if( mB.throttle <= 48 ) mB.throttle=48;
//		if( mC.throttle <= 48 ) mC.throttle=48;
//		if( mD.throttle <= 48 ) mD.throttle=48;
//	}
//	
//	if( keyid == PS2KEY_2RED )
//	{
//		mA.throttle=48;
//		mB.throttle=48;
//		mC.throttle=48;
//		mD.throttle=48;
//	}
	
	if( keyid == PS2KEY_START && event == GamePadKeyEvent_LONGCLICK )
	{
		StartFlag = !StartFlag;
	}
	else if( keyid == PS2KEY_START )
	{
		StartFlag = 0;
	}
	
}

//油门-电压-重量关系式
/**
 * @brief 由整机重量与当前电池电压拟合出悬停基础油门
 * @param weight 整机重量，单位 g
 * @param vol    当前电池电压，单位 V
 * @retval 悬停油门值（0~2047）
 * @note  使用实测数据二次多项式拟合，用于补偿电压下降导致的推力衰减。
 */
static uint16_t weight_to_throttle(float weight,float vol)
{
    return 1537.96f + 8.70f*weight - 225.2f*vol - 0.0088f*weight*weight - 0.24f*weight*vol + 9.01f*vol*vol;
}


//角度归一化处理,把角度限制在 -PI，PI
/**
 * @brief 将任意角度归一化到 [-PI, PI]
 * @param angle 输入角度（弧度）
 * @retval 归一化后的角度，用于航向误差计算避免 360° 跳变
 */
static float normalize_radian(float angle) {
	const float TWO_PI = 2.0f * PI; // 2π ≈ 6.283185307
	while (angle > PI) {
		angle -= TWO_PI;
	}
	while (angle < -PI) {
		angle += TWO_PI;
	}
	return angle;
}

/**
 * @brief 飞控主任务：以 200Hz 固定周期执行完整控制链
 * @param param 任务参数（FreeRTOS 传入，未使用）
 *
 * 单周期流程：
 *   1. 采集 9 轴 IMU 原始数据并解算欧拉角，采集电池电压与激光高度；
 *   2. 高度一阶低通微分求垂直速度；光流数据做旋转补偿与尺度缩放；
 *   3. 加速度积分 + 光流互补滤波融合出水平速度，梯形积分得到水平位移；
 *   4. 开机 10s 时执行一次 IMU/高度零点标定（蜂鸣器提示）；
 *   5. 读取控制队列（含 500ms 无指令失控保护），执行串级 PID 与 X 型混控；
 *   6. 输出 4 路 DShot 油门，更新调试监控数据。
 */
void Balance_Task(void *param)
{
	//获取时基,用于辅助任务能指定固定频率运行
	TickType_t preTime = xTaskGetTickCount();
	
	//控制任务的频率,单位HZ
	const uint16_t TaskFreq = 200;
	
	//标定零点定时器
	priv_BuzzerTipsTimer = xTimerCreate("BuzzerTips",pdMS_TO_TICKS(10),pdFALSE,NULL,BuzzerTipsTimer_Callback);
	
	//控制队列
	g_xQueueFlyControl = xQueueCreate(1,sizeof(FlyControlCMDType_t));
	
	//坐标数据队列
	g_xQueueFlyPosMsg = xQueueCreate(5,sizeof(FlyPosMsg_t));
	
	//用于接收控制队列的控制量
	FlyControlCMDType_t TargetCmd;
	
	//无队列统计
	uint16_t LostCmdCnt = 0;
	
	uint8_t WaitStartEvent = 0;
	
	//Z轴转向、高度的真实控制量
	float YawControl = 0;
	float HeightControl=0;
	
	//高度零点、高度与高度速度
	float zero_height = 0;
	float height = 0,height_dot=0;
	
	// 机体的x、y速度值
	float x_dot=0,y_dot=0;
	float x_dot_Prev = 0, y_dot_Prev = 0; 
	float x_pos=0,y_pos=0; 
	
	//坐标信息
	uint8_t PosMsgReady = 0;
	FlyPosMsg_t PosMsg;
	
	while(1)
	{
		//系统运行时间
		if( SystemRunTime < TaskFreq*30 ) SystemRunTime++;
		
		//获取9轴数据，存放在 imu_ori变量里
		imu->Update_9axisVal(&imu_ori);
		
		//计算欧拉角
		imu->UpdateAttitude(imu_ori,&attitude,(1.0f/(float)TaskFreq));
		
		//获取电压
		robot_vol = adc->getValue(userconfigADC_VBAT_CHANNEL);
		robot_vol = (robot_vol/4095.0f) * 3.3f * 11.0f; //电池的输入电压
		
		//获取高度信息
		height = zero_height - g_readonly_distance;
		static float last_height=0;
		static float last_height_dot = 0;
		height_dot = 0.4f*(height - last_height)*TaskFreq + 0.6f*last_height_dot; // 一阶低通滤波
		last_height_dot = height_dot;
		last_height = height;
		
		//进行数据融合前,对获取到的imu数据执行低通滤波
		static float gx_filtered,gy_filtered;
		gx_filtered = 0.9f * gx_filtered + 0.1f * imu_ori.gyro.x;
		gy_filtered = 0.9f * gy_filtered + 0.1f * imu_ori.gyro.y; 
		
		//读取坐标信息
		BaseType_t PosQueueState = xQueueReceive(g_xQueueFlyPosMsg,&PosMsg,0);	
		if( PosQueueState == pdPASS )
		{
			//标记存在位置队列
			PosMsgReady = 1;
			
			//光流部分
			PosMsg.x_speed = PosMsg.x_speed/200.0f; //将光流原始数据转换为 rad/s 单位
			PosMsg.y_speed = PosMsg.y_speed/200.0f;
			
			//旋转补偿 + 尺度缩放：光流像素速度换算为 rad/s 后，
			//叠加陀螺仪角速度补偿（限幅 ±2rad/s），再乘以当前高度得到机体水平速度 m/s
			// 旋转补偿 + 尺度缩放
			PosMsg.x_speed = height * (PosMsg.x_speed + fmaxf(fminf(gy_filtered,2.0f),-2.0f));
			PosMsg.y_speed = height * (PosMsg.y_speed - fmaxf(fminf(gx_filtered,2.0f),-2.0f));
		}
		
		//以下为 IMU 加速度与光流互补滤波融合：
		//1. 根据姿态角分解出重力在机体系的分量，得到运动加速度；
		// 计算重力分量
		float g = 9.8f;
		float g_x = -g * sin(attitude.pitch);
		float g_y =  g * sin(attitude.roll) * cos(attitude.pitch);
		float g_z =  g * cos(attitude.roll) * cos(attitude.pitch);

		// 计算运动加速度(机体坐标系)
		float a_x = imu_ori.accel.x - g_x;
		float a_y = imu_ori.accel.y - g_y;
		float a_z = imu_ori.accel.z - g_z;

		//2. 将运动加速度由机体坐标系旋转到世界坐标系；
		// 坐标变换(机体坐标系 --> 世界坐标系),计算的到机体加速度数据
		float a_motion_x = a_x * cos(attitude.pitch) + 
		                    a_y * sin(attitude.roll) * sin(attitude.pitch) + 
						    a_z * cos(attitude.roll) * sin(attitude.pitch);
		float a_motion_y =  a_y * cos(attitude.roll) - 
					         a_z * sin(attitude.roll);
		
		//平衡点上微小数值过滤
		if( fabs(a_motion_x)<0.15f ) a_motion_x=0;
		if( fabs(a_motion_y)<0.15f ) a_motion_y=0;
		
		//3. 互补滤波：加速度积分（高频可信）占 95% + 光流速度（低频可信）占 5%；
		//   融合得到平滑的水平速度，避免纯积分漂移；
		// 互补滤波融合imu与坐标的数据,最终得到x、y方向的速度
		x_dot = 0.95f * (x_dot + a_motion_x * (1.0f/(float)TaskFreq)) + 0.05f * PosMsg.x_speed; // 单位：m/s
		y_dot = 0.95f * (y_dot + a_motion_y * (1.0f/(float)TaskFreq)) + 0.05f * PosMsg.y_speed; // 单位：m/s

		//4. 梯形积分对速度积分得到水平位移，作为位置环反馈；
		// 使用梯形积分法计算位移
		x_pos += (x_dot + x_dot_Prev) * 0.5f * (1.0f/(float)TaskFreq); // 单位：m
		y_pos += (y_dot + y_dot_Prev) * 0.5f * (1.0f/(float)TaskFreq); // 单位：m
		
		//保存本次速度
		x_dot_Prev = x_dot;
		y_dot_Prev = y_dot;
		
		//当前飞行器没有位置相关的设备,将位置信息清空
		if( PosMsgReady == 0 )
		{
			x_dot=0;y_dot=0;
			x_pos=0;y_pos=0;
			x_dot_Prev=0;y_dot_Prev=0;
		}
		
		//进阶写法：开机后,持续判断IMU是否静止,如果静止一段时间,则采集某段的时间的imu数据取平均,进行零点标定
		//开机10秒后,执行1次零点标定
		if( SystemRunTime == TaskFreq*10 )
		{
			memcpy(&zero_imu_ori,&imu_ori,sizeof(IMU_DATA_t));
			zero_imu_ori.accel.z -= 9.8f;
			imu->UpdateZeroPoint_axis(&zero_imu_ori);
			 
			//只标定了yaw方向的
			memcpy(&zero_attitude,&attitude,sizeof(ATTITUDE_DATA_t));
			zero_attitude.pitch=0;
			zero_attitude.roll=0;
			imu->UpdateZeroPoint_attitude(&zero_attitude);
			
			//获取高度的零点
			zero_height = g_readonly_distance;
				
			//标零提示
			xTimerStart(priv_BuzzerTipsTimer,0);
		}
		
		//检查控制队列是否有数据
		BaseType_t CmdQueueState = xQueueReceive(g_xQueueFlyControl,&TargetCmd,0);	
	
		if( CmdQueueState == pdPASS )
		{
			LostCmdCnt=0;
			YawControl += TargetCmd.yaw; //接收航向角命令
			HeightControl += TargetCmd.height;//接收高度控制指令
			
			//高度限制
			static uint8_t tips = 1;
			if( HeightControl > MaxHeight)
			{
				HeightControl = MaxHeight;
				if( tips ) tips=0,xTimerStart(Timer_FailTips,0);
			}
			else if( HeightControl < MinHeight )
			{
				HeightControl = MinHeight;
				if( tips ) tips=0,xTimerStart(Timer_FailTips,0);
			}
			else
			{
				tips=1;
			}
			
		}
		else 
		{
			//失控保护,队列持续无数据500ms时清空目标指令
			LostCmdCnt++;
			if( LostCmdCnt >= TaskFreq/2 )
			{
				memset(&TargetCmd,0,sizeof(TargetCmd));
			}
		}
		
		//启动飞行器
		if( StartFlag==1 )
		{
			if( WaitStartEvent==1 )
			{
				WaitStartEvent++;
				YawControl = attitude.yaw;
			}
			else if( WaitStartEvent==2 )
			{
				//存在控制量时,跳过开机自增
				if( TargetCmd.height!=0 ) WaitStartEvent++;
				
				//以0.5m/s到达指定位置
				if( HeightControl < DefalutHeight )
				{
					HeightControl+= (0.5f/(float)TaskFreq);
				}
				else WaitStartEvent++;
				
				//高度较小时保持位置清空
				if( height < 0.3f )
				{
					x_dot=0;y_dot=0;
					x_pos=0;y_pos=0;
					x_dot_Prev=0;y_dot_Prev=0;
				}
			}
			
			//位置环（外环）：PD 控制，输入为 0 与位移/速度误差，输出限制在 ±0.35rad；
			//输出作为目标角度叠加到角度环（TargetCmd.roll/pitch ± controlY/controlX）
			//位置环-X、Y方向
			const float limitPos = 0.35f;
			float controlX = 0.5f * (0 - x_pos) - 0.7f * x_dot; //PD公式
			float controlY = 0.5f * (0 - y_pos) - 0.7f * y_dot;
			
			if( controlX > limitPos ) controlX = limitPos;
			if( controlX <-limitPos ) controlX = -limitPos;
			if( controlY > limitPos ) controlY = limitPos;
			if( controlY <-limitPos ) controlY = -limitPos;
			
			//控制时清空位置
			if( TargetCmd.pitch != 0 || TargetCmd.roll != 0  )
			{
				controlX=0;controlY=0;
				x_dot=0;y_dot=0;
				x_pos=0;y_pos=0;
				x_dot_Prev=0;y_dot_Prev=0;
			}
			
			//串级 PID 外环：角度环（roll/pitch/yaw/height），输出为目标角速度
			//roll-外环(角度)
			PID_Update(&rollPID,TargetCmd.roll - controlY,attitude.roll);
			
			//pitch-外环(角度)
			PID_Update(&pitchPID,TargetCmd.pitch + controlX,attitude.pitch);
			
			//yaw-外环(角度)
			float yawerr = normalize_radian(YawControl-attitude.yaw);
			PID_Update(&yawPID,yawerr,0);
			
			//height-外环(高度)
			PID_Update(&heightPID,HeightControl,height);
			
			//串级 PID 内环：角速度环（rollrate/pitchrate/yawrate），输出直接参与电机混控
			//rollrate-内环(角速度)
			PID_Update(&rollratePID,rollPID.output,imu_ori.gyro.x);
			
			//pitchrate-内环(角速度)
			PID_Update(&pitchratePID,pitchPID.output,imu_ori.gyro.y);
			
			//yawrate-内环(角速度)
			PID_Update(&yawratePID,yawPID.output,imu_ori.gyro.z);
			
			//height-speed-内环(高度的速度)
			PID_Update(&heightSpeedPID,heightPID.output,height_dot);
			
			//X 型四旋翼混控：
			//基础油门 + 内环输出组合，roll/pitch 差动、yaw 差动、高度速度叠加到 4 个电机
			//获取基础油门
			float basethro = weight_to_throttle(140.0f,robot_vol);
			
			//pid的输出
			mA.throttle = basethro - rollratePID.output - pitchratePID.output + yawratePID.output + heightSpeedPID.output;
			mB.throttle = basethro + rollratePID.output - pitchratePID.output - yawratePID.output + heightSpeedPID.output;
			mC.throttle = basethro + rollratePID.output + pitchratePID.output + yawratePID.output + heightSpeedPID.output;
			mD.throttle = basethro - rollratePID.output + pitchratePID.output - yawratePID.output + heightSpeedPID.output;
			
			//用户通过高度关停飞行器
			static uint16_t StopCnt = 0;
			if( HeightControl == MinHeight && TargetCmd.height<0 && height <= 0.05f )
			{
				StopCnt++;
				if( StopCnt > TaskFreq ) StopCnt=0,StartFlag=0;
			}
			else StopCnt = 0;
			
			//判断角度是否超出限幅
			if( fabs(attitude.pitch) > 40*0.017453292f || fabs(attitude.roll) > 40*0.017453292f )
			{
				StartFlag=0;//主动关闭电机
			}
			
		}
		else
		{
			//飞行器停止时,清空积分项的累计
			PID_Reset(&rollPID);
			PID_Reset(&rollratePID);
			PID_Reset(&pitchPID);
			PID_Reset(&pitchratePID);
			PID_Reset(&yawPID);
			PID_Reset(&yawratePID);
			PID_Reset(&heightPID);
			PID_Reset(&heightSpeedPID);
			
			//停止电机
			mA.throttle = 48;
			mB.throttle = 48;
			mC.throttle = 48;
			mD.throttle = 48;
			
			//等待下次启动
			WaitStartEvent = 1;
			
			//高度控制设置为0,等待下一次启动
			HeightControl = 0;
		}
		
		//输出 4 路油门到电调（注意：停机时输出 48 最低油门用于解锁/待机）
		//发送油门指令到电调(控制前,先发送最低油门一段时间进行解锁)
		dshot->set_target(mA,mD,mC,mB);
		
		//对外监控数据
		debugshow.gyrox = imu_ori.gyro.x;
		debugshow.gyroy = imu_ori.gyro.y;
		debugshow.gyroz = imu_ori.gyro.z;
		debugshow.pitch = attitude.pitch;
		debugshow.roll = attitude.roll;
		debugshow.yaw = attitude.yaw;
		debugshow.height = height;
		debugshow.heightSpeed = height_dot;
//		debugshow.x_dot = x_dot;
//		debugshow.y_dot = y_dot;
//		debugshow.x_pos = x_pos;
//		debugshow.y_pos = y_pos;
		
//		debugshow.targetRoll = roll_control;
//		debugshow.targetPitch = pitch_control;
//		debugshow.targetYaw = YawControl;
//		debugshow.targetHeight = 0;
		
		/* 延迟指定频率 */
		vTaskDelayUntil(&preTime,pdMS_TO_TICKS( (1.0f/(float)TaskFreq)*1000) );
	}
	
}

/**
 * @brief  此函数用于写控制队列,所有需要控制飞行器的任务均需要通过此任务写入控制量.
 * @param  cmd:参数需要包含指令来源,以及指定飞行器的控制量
           woken:如果是在中断中调用,需要传入此参数。非中断中调用直接写入0
 * @retval 0写入成功,1写入失败.写入失败原因：未指定控制指令来源或当前有更高优先级控制在活跃
 *
 * @details
 * - 用户需要新增控制方式时,需要先在balance.h头文件新增一个控制类型的枚举并确定优先级,
 *   然后通过 WriteFlyControlQueue 写控制队列进行小车的控制
 */
uint8_t WriteFlyControlQueue(FlyControlCMDType_t* cmd,BaseType_t* woken)
{
	static uint32_t lastTick = 0;
	uint8_t writeflag = 0;
	
	//队列未创建或未指定的控制源,忽略写入
	if(g_xQueueFlyControl == NULL || cmd->cmdsource == NONE_CMD || 
		cmd->cmdsource >= UNKNOWN_CMD ) return 1;
	
	//高优先级的控制指令,可直接抢占控制
	if( cmd->cmdsource <= FlyControl_CMDsource )
	{
		FlyControl_CMDsource = cmd->cmdsource;
		writeflag = 1;
	}
	else
	{
		//接受到低优先级控制指令,检查活跃时间戳
		uint32_t tick = 0;
		if( __get_IPSR()!=0 ) tick = xTaskGetTickCountFromISR();
		else tick = xTaskGetTickCount();
		
		//高优先级指令超2000ms未活跃,则允许低优先级指令执行
		if( tick - lastTick >= 2000 )
		{
			FlyControl_CMDsource = cmd->cmdsource;
			writeflag = 1;
		}
	}
	
	//允许写入队列
	if( writeflag )
	{
		if( __get_IPSR()!=0 ) //中断上下文判断
		{
			lastTick = xTaskGetTickCountFromISR();
			xQueueOverwriteFromISR(g_xQueueFlyControl,cmd,woken);
		}			
		else 
		{
			lastTick = xTaskGetTickCount();
			xQueueOverwrite(g_xQueueFlyControl,cmd);
		}
		return 0;
	}
	
	return 1;
}


//标定完成蜂鸣器提示定时器
/**
 * @brief 零点标定完成蜂鸣器提示（成功音）
 * @param xTimer 触发本回调的软件定时器（未使用）
 */
static void BuzzerTipsTimer_Callback(TimerHandle_t xTimer)
{
	pBuzzeInterface_t tips = &UserBuzzer;
	tips->on();
	vTaskDelay(200);
	tips->off();
	vTaskDelay(300);
	for(uint8_t i=0;i<30;i++)
	{
		tips -> toggle();
		vTaskDelay(25);
	}
	tips->off();
}


