/**
 ******************************************************************************
 * @file	bsp_dwt.h
 * @author  Wang Hongxi
 * @version V1.1.0
 * @date    2022/3/8
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef BSP_DWT_H
#define BSP_DWT_H


#include "stdint.h"
/**
 * @brief 微秒级延时函数
 * @param us 延时微秒数
 * @note 该函数基于DWT实现，确保SysTick定时器已初始化
 */
void Dwt_delay_us(uint16_t us);
/**
 * @brief 毫秒级延时函数
 * @param ms 延时毫秒数
 * @note 该函数调用Dwt_delay_us实现毫秒级延时
 */
void Dwt_delay_ms(uint16_t ms);

#endif /* BSP_DWT_H */
