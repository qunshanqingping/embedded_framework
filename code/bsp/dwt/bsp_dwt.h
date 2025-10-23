#ifndef BSP_DWT_H
#define BSP_DWT_H

#include "stdint.h"
/**
 * @brief DWT延时初始化函数
 * @note 该函数用于初始化DWT计数器，必须在使用延时函数前调用
 */

void Dwt_Init(void);
/**
 * @brief 微秒级延时函数
 * @param us 延时微秒数
 */

void Dwt_delay_us(const uint16_t us);
/**
 * @brief 毫秒级延时函数
 * @param ms 延时毫秒数
 */

void Dwt_delay_ms(const uint16_t ms);
#endif /* BSP_DWT_H */
