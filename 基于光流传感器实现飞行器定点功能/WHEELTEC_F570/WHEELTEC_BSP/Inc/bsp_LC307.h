/**
 * @file    bsp_LC307.h
 * @brief   LC307 光流传感器接口定义
 * 包含 DMA 原始缓冲区、单帧光流数据结构体（1 字节对齐）与接口声明。
 */
#ifndef __BSP_LC307_H
#define __BSP_LC307_H

#include <stdint.h>

//LC307光流传感器 DMA数据缓冲区分配
#define userconfig_LC307_DMABUF_LEN 50

//stp23L DMA原始数据存放结构体
typedef struct {
	uint8_t buffer[userconfig_LC307_DMABUF_LEN];
	uint16_t DataLen;
}OriData_LC307_t;


//1帧光流数据内容
#pragma pack(1)
typedef struct{
	uint8_t head1;
	uint8_t bufcount;
	short flowX;
	short flowY;
	uint16_t timespan;
	uint16_t distance;
	uint8_t quality;
	uint8_t version;
	uint8_t XORSum;
	uint8_t end;
}OpticalFlowFrame_t;
#pragma pack()

extern OriData_LC307_t DMABuf_oridata_LC307;
uint8_t XOR_Checksum(const uint8_t *data, uint16_t length);
uint8_t Opf_LC307_Init(void);
#endif

