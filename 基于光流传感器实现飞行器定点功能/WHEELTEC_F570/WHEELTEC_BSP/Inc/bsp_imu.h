/**
 * @file    bsp_imu.h
 * @brief   IMU 数据与操作接口定义
 * 定义九轴原始数据/姿态角结构体，以及 IMU 初始化、零位标定、
 * 数据更新、姿态解算的函数指针接口。
 */
#ifndef __BSP_IMU_H
#define __BSP_IMU_H

#include <stdint.h>

typedef struct{ //中间私有变量
	float x;
	float y;
	float z;
}PrivateBuf_t;
	
typedef struct{
	PrivateBuf_t gyro; //3轴角速度
	PrivateBuf_t accel;//3轴加速度
	PrivateBuf_t magn;//3轴磁力计
}IMU_DATA_t;

typedef struct{
	float roll;
	float pitch;
	float yaw;
}ATTITUDE_DATA_t;


//IMU操作接口
typedef struct {
	uint8_t (*Init)(void);   //返回值：1错误 0无异常
	uint8_t (*DeInit)(void); //返回值：1错误 0无异常

	void (*UpdateZeroPoint_axis)(const IMU_DATA_t* point);               //更新9轴数据零点
	void (*UpdateZeroPoint_attitude)(const ATTITUDE_DATA_t* attitude);   //更新姿态零点
	void (*Update_9axisVal)(IMU_DATA_t* imudata);                         //更新9轴数据
	void (*UpdateAttitude)(IMU_DATA_t imudata,ATTITUDE_DATA_t *attitude,float dt); //更新姿态角
	
}IMUInterface_t,*pIMUInterface_t;

extern IMUInterface_t UserICM20948;

#endif /* __BSP_IMU_H */
