/**
 * @file    bsp_stp23L.h
 * @brief   STP23L 激光雷达接口定义
 * 包含 DMA 原始缓冲区、点云/整帧数据结构体（1 字节对齐）与解析接口声明。
 */
#ifndef __BSP_STP23L_H
#define __BSP_STP23L_H

#include <stdint.h>

//stp23L传感器 DMA数据缓冲区分配
#define userconfig_STP23L_DMABUF_LEN 200

//stp23L DMA原始数据存放结构体
typedef struct {
	uint8_t Buf[userconfig_STP23L_DMABUF_LEN];
	uint16_t DataLen;
}OriData_STP23L_t;

//1个点云数据包里包含的信息。一帧数据共有12个点云数据
#pragma pack(1)
typedef struct { 
    int16_t distance;   //距离
    uint16_t noise;     //环境噪声
    uint32_t peak;      //强度
    uint8_t confidence; //置信度
    uint32_t intg;      //积分次数
    int16_t reftof;     //温度表征值
}LidarPointTypedef; 

typedef struct { 
    uint32_t Head;      //帧头,4个0xAA
		uint8_t dev_addr;   //设备地址
		uint8_t cmdcode;    //命令码
		uint16_t diff_addr; //偏移地址
		uint16_t datalen;   //数据长度
		LidarPointTypedef point[12];//12个数据测量点
		uint32_t timestamp;  //时间戳
		uint8_t crc_code;    //校验码
}STP23LSensorTypedef;

#pragma pack()

extern OriData_STP23L_t DMABuf_oridata_stp23L;

uint8_t stp23L_callback(OriData_STP23L_t* buffer);
extern float g_readonly_distance ;

#endif /* __BSP_STP23L_H */


