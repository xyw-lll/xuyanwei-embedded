/**
 * @file    bsp_imu.c
 * @brief   ICM20948 九轴 IMU 驱动与姿态解算
 *
 * 功能：
 *   - I2C 寄存器级初始化 ICM20948（陀螺/加速度/磁力计量程与滤波配置）；
 *   - 读取九轴原始数据并换算为物理单位（rad/s、m/s^2、uT），去除零点漂移；
 *   - Mahony 互补滤波（四元数）姿态解算：陀螺积分 + 加速度/磁力计修正，
 *     输出欧拉角 roll/pitch/yaw，并支持开机零位校正。
 */
#include "bsp_imu.h"

//ICM20948寄存器
#include "icm20948_reg.h"

//imu需要iic支持包
#include "bsp_iic.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

/**
 * @brief ICM20948 初始化：寄存器配置（量程、滤波、采样率、磁力计）
 * @retval 0 成功；1 失败（WHO_AM_I 校验错误或 I2C 通信错误）
 */
static uint8_t ICM20948_Init(void)
{
	pIICInterface_t iicdev = &UserII2Dev;//初始化iic设备
	
//	//IIC初始化
//	iicdev->init();
	
	IIC_Status_t check_state = IIC_OK;//检查初始化步骤是否出错
	
	uint8_t writebuf = 0;
	
	//选择bank0
	writebuf = REG_VAL_SELECT_BANK_0;
	check_state += iicdev -> write_reg(ICM20948_DEV<<1,REG_BANK_SEL,&writebuf,1,500);
	
	//访问icm20948 WHO AM I寄存器,检查设备是否正确
	check_state += iicdev -> read_reg(ICM20948_DEV<<1,WHO_AM_I,&writebuf,1,500);
	if( 0xEA != writebuf ) return 1;//数据错误
	
	//操作PWR1复位icm20948
	writebuf = (1<<7);
	check_state += iicdev -> write_reg(ICM20948_DEV<<1,PWR_MGMT_1,&writebuf,1,500);
	iicdev -> delay_ms(100); //延迟等待复位完毕
	
	//配置user_ctrl
	// 不开启dmp: 0<<7
	// 不开启fifo: 0<<6
	// 不开启icm20948内部的iic读取数据 0<<5
	// 不复位从机iic：0 << 4
	// 不复位dmp    : 0<<3
	// 不复位sram模块：0 << 2
	// 不复位iic主机摸：0 << 1
	writebuf = 0x00;
	check_state += iicdev -> write_reg(ICM20948_DEV<<1,USER_CTRL,&writebuf,1,500);
	
	//关闭低功耗模式,使能温度传感器,设置自动选择最佳时钟源
	writebuf = 0x01;
	check_state += iicdev -> write_reg(ICM20948_DEV<<1,PWR_MGMT_1,&writebuf,1,100);
	
	//选择bank2
	writebuf = REG_VAL_SELECT_BANK_2;
	check_state += iicdev -> write_reg(ICM20948_DEV<<1,REG_BANK_SEL,&writebuf,1,100);
	
	//设置频率GYRO数据输出的频率（ODR:output data rate = 1.1KHz / 1 + 配置值 ）
	writebuf = 0x04; //设置GYRO的采样频率为1.1KHz（需要FCHOICE=1(启用滤波功能),DLPF_CFG=1~7(配置滤波的信息)）
	check_state += iicdev -> write_reg(ICM20948_DEV<<1,GYRO_SMPLRT_DIV,&writebuf,1,100);
	
	//配置 GYRO_CONFIG_1，配置DLPCFG，配置量程，配置DLFP
	//FS_SEL[2:1]陀螺仪量程配置：（3<<1：正负2000dps 2<<1:正负1000dps 1<<1:正负500dps 0<<1:正负250dps）
	//FCHOICE[0]滤波使能 ： 1 --> 开启滤波
	//DLPFCFG[5:3]滤波情况配置：5 （3DB BW:11.6Hz NBW:17.8Hz）3DB BW:频率越高，陀螺仪响应速度越快，但噪点可能增加 NBW:频率越低，数据越稳定
	writebuf = (3<<1) | (1<<0) | (3<<3);
	check_state += iicdev -> write_reg(ICM20948_DEV<<1,GYRO_CONFIG_1,&writebuf,1,100);
	
	//设置频率ACCEL_SMPLRT_DIV_2(输出频率配置：1.125kHz/(1+配置值))
	writebuf = 0x04;
	check_state += iicdev -> write_reg(ICM20948_DEV<<1,ACCEL_SMPLRT_DIV_2,&writebuf,1,100);
	
	//ACCEL_DLPFCFG[5:3]滤波配置：
	//FS_SEL[2:1]加速度量程：（3<<1:正负16g 2<<1:正负8g 1<<1:正负4g 0<<1:正负2g）
	//FCHOICE[0] 滤波使能位:
	writebuf = (0<<1) | (1<<0) | (5<<3);
	check_state += iicdev -> write_reg(ICM20948_DEV<<1,ACCEL_CONFIG,&writebuf,1,100);
	
	//选择回bank0
	writebuf = REG_VAL_SELECT_BANK_0;
	check_state += iicdev -> write_reg(ICM20948_DEV<<1,REG_BANK_SEL,&writebuf,1,100);
	
	//配置磁力计
	
	//启动icm20948的bypass 旁路模式，允许直接通过IIC总线访问磁力计而不需要经过icm20948
	writebuf = (1<<1);
	check_state += iicdev -> write_reg(ICM20948_DEV<<1,INT_PIN_CFG,&writebuf,1,100);
	
	//访问磁力计，确认设备是否正确
	check_state += iicdev -> read_reg(AK09916_DEV<<1,WIA,&writebuf,1,100);
	if( 0x09 != writebuf ) return 1;//数据错误返回
	
	/* 配置磁力计输出频率
	doc:
	0:关闭输出
	1<<0:单次测量模式,测量后自动回到关闭输出模式
	1<<1:连续测量模式1，频率10Hz
	1<<2:连续测量模式2，频率20Hz
	(1<<2)|(1<<1):连续测量模式3，频率50Hz
	1<<3:连续测量模式4，频率100Hz
	*/
	writebuf = (1<<3);//配置磁力计为100hz输出
	check_state += iicdev -> write_reg(AK09916_DEV<<1,CNTL2,&writebuf,1,100);
	//磁力计量程说明： ±4900 uT , 分辨率16位
	
	//初始化过程中有错误
	if( 0 != check_state ) return 1;
	
	return 0;
}

/**
 * @brief ICM20948 反初始化：进入睡眠模式
 * @retval 0 成功；1 失败
 */
static uint8_t ICM20948_DeInit(void)
{
	pIICInterface_t iicdev = &UserII2Dev;//初始化iic设备
	
	IIC_Status_t check_state = IIC_OK;//检查初始化步骤是否出错
	
	uint8_t writebuf = 0;
	
	//选择bank0
	writebuf = REG_VAL_SELECT_BANK_0;
	check_state += iicdev -> write_reg(ICM20948_DEV<<1,REG_BANK_SEL,&writebuf,1,500);
	
	//访问icm20948 WHO AM I寄存器,检查设备是否正确
	check_state += iicdev -> read_reg(ICM20948_DEV<<1,WHO_AM_I,&writebuf,1,500);
	if( 0xEA != writebuf ) return 1;//数据错误
	
	//设置icm20948进入睡眠模式
	writebuf = (1<<6);
	check_state += iicdev -> write_reg(ICM20948_DEV<<1,PWR_MGMT_1,&writebuf,1,500);
	
	//过程中有错误
	if( 0 != check_state ) return 1;
	
	return 0;
}


static IMU_DATA_t ZeroPoint = { 0 };//零点
static ATTITUDE_DATA_t ZeroAttitude = { 0 };


/**
 * @brief 读取九轴数据并换算物理单位、去除零漂
 * @param data 输出：加速度(m/s^2)、角速度(rad/s)、磁力计(uT)
 */
static void ImuUpdate(IMU_DATA_t *data)
{
	pIICInterface_t iicdev = &UserII2Dev;//指定iic设备
	
	//获得6轴的数据原始值
	uint8_t tmpbuf[12];
	iicdev -> read_reg(ICM20948_DEV<<1,ACCEL_XOUT_H,tmpbuf,12,100);
	
	//获得加速度3轴原始值
	data->accel.x = (short)(tmpbuf[0]<<8 | tmpbuf[1]);
	data->accel.y = (short)(tmpbuf[2]<<8 | tmpbuf[3]);
	data->accel.z = (short)(tmpbuf[4]<<8 | tmpbuf[5]);
	
	//将单位转换为 m/s^2。 量程 正负2g，分辨率 2^16
	data->accel.x *= 0.00059814453125f;
	data->accel.y *= 0.00059814453125f;
	data->accel.z *= 0.00059814453125f;
	
	//获得角速度3轴原始值
	data->gyro.x = (short)(tmpbuf[6]<<8 | tmpbuf[7]);
	data->gyro.y = (short)(tmpbuf[8]<<8 | tmpbuf[9]);
	data->gyro.z = (short)(tmpbuf[10]<<8 | tmpbuf[11]);
	
	//将单位转换为 rad/s。 量程 正负2000dps，分辨率 2^16
	data->gyro.x *= 0.06103515625f;//单位dps
	data->gyro.y *= 0.06103515625f;
	data->gyro.z *= 0.06103515625f;
	
	data->gyro.x *= 0.01745329252f;//单位rad/s
	data->gyro.y *= 0.01745329252f;
	data->gyro.z *= 0.01745329252f;
	
	//去除零点漂移
	data->gyro.x -= ZeroPoint.gyro.x; 
	data->gyro.y -= ZeroPoint.gyro.y;
	data->gyro.z -= ZeroPoint.gyro.z;
	
	//获得3轴磁力计数据
	uint8_t magnbuf[8]; //byte[6]作废,地址0x17无数据。byte[7]为寄存器ST2,地址0x18. 为了加快效率使用连续方式读取
	iicdev->read_reg(AK09916_DEV<<1,HXL,magnbuf,8,100);//读取磁力计数据
	
	if( 0 == ((magnbuf[7]>>3)&0x01) ) 
	{
		//获得磁力计3轴原始值
		data->magn.x = (short)(magnbuf[1]<<8 | magnbuf[0]);
		data->magn.y = (short)(magnbuf[3]<<8 | magnbuf[2]);
		data->magn.z = (short)(magnbuf[5]<<8 | magnbuf[4]);
		
		//将单位转换为uT。量程正负4900uT，分辨率 2^16
		data->magn.x *= 0.1495361328125f;
		data->magn.y *= 0.1495361328125f;
		data->magn.z *= 0.1495361328125f;
	}
	else
	{
		//数据不可靠,不处理,保留上一次的数据
	}
}

//标定零点的方法
/**
 * @brief 保存九轴零点（开机标定：静止时采集并取均值）
 * @param point 标定的零点数据
 */
static void setZeroPoint_axis(const IMU_DATA_t* point) 
{
	memcpy(&ZeroPoint,point,sizeof(IMU_DATA_t));
}

/**
 * @brief 保存姿态零点（开机标定：当前朝向作为 yaw 零位）
 * @param attitude 标定的姿态零点
 */
static void setZeroPoint_attitude(const ATTITUDE_DATA_t* attitude)
{
	memcpy(&ZeroAttitude,attitude,sizeof(ATTITUDE_DATA_t));
}

static const float Kp = 0.8;   // 比例系数
static const float Ki = 0.0001; // 积分系数

//快速计算 1/sqrt(x)
/**
 * @brief 快速计算 1/sqrt(x)（魔数法 + 一次牛顿迭代）
 * @param number 输入数值
 * @retval 1/sqrt(number) 近似值，用于归一化运算加速
 */
static float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;
 
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                      
	i  = 0x5f3759df - ( i >> 1 );               
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration （第一次牛顿迭代）
	return y;
} 

/**
 * @brief Mahony 互补滤波姿态解算（四元数法）
 * @param imudata  九轴输入（角速度 rad/s、加速度 m/s^2、磁力计 uT）
 * @param attitude 输出欧拉角（roll/pitch/yaw，弧度）
 * @param dt       解算周期（s）
 * @note  原理：陀螺积分姿态 + 加速度/磁力计估计重力方向做叉积误差，
 *        PI 修正角速度后更新四元数，最后四元数转欧拉角并扣除零位。
 */
static void attitudeUpdate(IMU_DATA_t imudata, ATTITUDE_DATA_t *attitude,float dt) // 显示互补滤波（陀螺仪 + 加速度计）
{
	static float eInt[3] = {0}; // 初始化累计误差
	static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 初始化四元数
	float norm;
	float vx, vy, vz;
	float ex, ey, ez; // 误差变量
	float q0_t,q1_t,q2_t,q3_t;
	
	float halftime = dt * 0.5f; //采样周期
	
	// 辅助变量（简化变量名）
	float gx = imudata.gyro.x;  // 绕X轴旋转的角速度(rad/s)
	float gy = imudata.gyro.y;  // 绕Y轴旋转的角速度(rad/s)
	float gz = imudata.gyro.z;  // 绕Z轴旋转的角速度(rad/s)
	float ax = imudata.accel.x; // 沿X轴移动的加速度(m·s^-2)
	float ay = imudata.accel.y; // 沿Y轴移动的加速度(m·s^-2)
	float az = imudata.accel.z; // 沿Z轴移动的加速度(m·s^-2)

	// 辅助变量（避免冗长的计算表达式）
	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;
	
	// 提取等效旋转矩阵中的重力分量 
	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = 1-2*(q1q1+q2q2);
	
	//加速度计归一化
	norm = Q_rsqrt(ax * ax + ay * ay + az * az);
	ax *= norm;
	ay *= norm;
	az *= norm;

	// 计算误差
	ex = ay * vz - az * vy;
	ey = az * vx - ax * vz;
	ez = ax * vy - ay * vx;

	// 累计误差
	eInt[0] += ex;
	eInt[1] += ey;
	eInt[2] += ez;

	// PI控制
	gx += Kp * ex + Ki * eInt[0];
	gy += Kp * ey + Ki * eInt[1];
	gz += Kp * ez + Ki * eInt[2];

	// 更新四元数
	q0_t =  (-q1 * gx - q2 * gy - q3 * gz) * halftime;
	q1_t =  ( q0 * gx + q2 * gz - q3 * gy) * halftime;
	q2_t =  ( q0 * gy - q1 * gz + q3 * gx) * halftime;
	q3_t =  ( q0 * gz + q1 * gy - q2 * gx) * halftime;
	
	q0+=q0_t;
	q1+=q1_t;
	q2+=q2_t;
	q3+=q3_t;
	
	// 归一化（四元数）
	norm = Q_rsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= norm;
	q1 *= norm;
	q2 *= norm;
	q3 *= norm;

	// 四元数转换成欧拉角
	attitude -> roll  = atan2f(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1);
	attitude -> pitch = asin(-2 * q1 * q3 + 2 * q0 * q2);
	attitude -> yaw   = atan2(2 *(q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);

	// 零点漂移矫正
	attitude -> roll  -= ZeroAttitude.roll ;
	attitude -> pitch -= ZeroAttitude.pitch ;
	attitude -> yaw   -= ZeroAttitude.yaw;

}


//驱动挂载
IMUInterface_t UserICM20948 = {
	.Init = ICM20948_Init,
	.DeInit = ICM20948_DeInit,
	.UpdateZeroPoint_axis = setZeroPoint_axis,
	.UpdateZeroPoint_attitude = setZeroPoint_attitude,
	.Update_9axisVal = ImuUpdate,
	.UpdateAttitude = attitudeUpdate
};
