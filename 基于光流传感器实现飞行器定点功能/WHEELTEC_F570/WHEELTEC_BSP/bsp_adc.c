/**
 * @file    bsp_adc.c
 * @brief   电压/电流采集驱动（ADC1 + DMA）
 *
 * 通过 DMA 循环搬运 ADC 采样值到缓冲数组，提供多通道滑动平均
 * 读取接口，供飞控计算电池电压（电压分压 11 倍）与回路电流。
 */
#include "bsp_adc.h"
#include "adc.h"

float g_robotVOL = 12.0f;

#define userconfig_ADCDMA_BUF_LEN 80 //设置dma搬运某个通道的个数

static uint16_t g_Adc1Buf[userconfig_ADCDMA_BUF_LEN][2] = { 0 };//存放ADC数据的数组

/**
 * @brief 启动 ADC 采集并配置 DMA 传输
 * @note  双通道循环采样，缓冲区大小 80 组 x 2 通道
 */
void ADC_Userconfig_Init(void)
{
	//启动ADC采集并设置DMA传输
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)g_Adc1Buf,sizeof(g_Adc1Buf)/sizeof(g_Adc1Buf[0][0]));
}

//获取DMA搬运后的ADC值
/**
 * @brief 获取指定通道的 ADC 均值（80 次采样平均，抑制噪声）
 * @param channel 通道号 0/1
 * @retval 0~4095 的均值；通道号非法时返回 0
 */
uint16_t USER_ADC_Get_AdcBufValue(uint8_t channel)
{
	uint32_t tmp = 0;
	
	//防止索引溢出
	if( channel > 2 ) return 0;
	
	for(uint8_t i=0;i<userconfig_ADCDMA_BUF_LEN;i++)
	{
		tmp += g_Adc1Buf[i][channel];
	}
	
	return tmp/userconfig_ADCDMA_BUF_LEN;
}

ADCInterface_t UserADC1 = {
	.init = ADC_Userconfig_Init,
	.getValue = USER_ADC_Get_AdcBufValue,
};

