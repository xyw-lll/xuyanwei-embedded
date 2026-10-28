/**
 * @file    bsp_buzzer.c
 * @brief   蜂鸣器驱动（GPIO 电平控制）
 * 提供 on/off/toggle 三接口，供操作成功/失败提示定时器使用。
 */
#include "bsp_buzzer.h"
#include "gpio.h"

/**
 * @brief 蜂鸣器响
 */
static void BuzzerOn(void)
{
    HAL_GPIO_WritePin(UserBuzzer_GPIO_Port,UserBuzzer_Pin,GPIO_PIN_SET);
}

/**
 * @brief 蜂鸣器停
 */
static void BuzzerOff(void)
{
    HAL_GPIO_WritePin(UserBuzzer_GPIO_Port,UserBuzzer_Pin,GPIO_PIN_RESET);
}

/**
 * @brief 蜂鸣器翻转（用于短促提示音）
 */
static void Buzzeroggle(void)
{
    HAL_GPIO_TogglePin(UserBuzzer_GPIO_Port,UserBuzzer_Pin);
}

BuzzerInterface_t UserBuzzer = {
    .on = BuzzerOn,
    .off = BuzzerOff,
    .toggle = Buzzeroggle
};

