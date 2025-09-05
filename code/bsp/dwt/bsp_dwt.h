/**
* @file bsp_dwt.c
 * @brief 更精确的延时函数和系统时间功能实现
 * @author Adonis Jin
 * @date 2025-7-27
 * @version 1.0
 * @note  实现基本功能
 * @date 2025-8-26
 * @version 1.1
 * @note 规范命名；修改Dwt_Init函数，无需传入频率参数；修改延时函数，不使用float进行时间计算
 */
#ifndef __BSP_DWT_H
#define __BSP_DWT_H

#include "stdint.h"

/**
 * @brief 系统时间结构体定义
 */
typedef struct
{
    uint32_t s;   // 秒
    uint32_t ms;  // 毫秒
    uint32_t us;  // 微秒
} DwtTime_s;

/**
 * @brief 初始化DWT（Data Watchpoint and Trace）模块
 * @note  使能DWT CYCLIST计数器，并根据CPU频率设置相关变量
 */
void Dwt_Init(void);

/**
 * @brief 获取两个时间点之间的时间差(单精度浮点数)
 * @note  计算从上次记录的时间点到当前时间点经过的时间，单位为秒
 * @param cnt_last 指向记录上次计数值的指针
 * @return float 时间差，单位为秒
 */
float Get_Time_Delta(uint32_t *cnt_last);

/**
 * @brief 获取两个时间点之间的时间差(双精度浮点数)
 * @note  计算从上次记录的时间点到当前时间点经过的时间，单位为秒
 * @param cnt_last 指向记录上次计数值的指针
 * @return double 时间差，单位为秒
 */
double Get_Time_Delta64(uint32_t *cnt_last);

/**
 * @brief 更新系统时间
 * @note  根据DWT计数器的值更新系统时间结构体，包括秒、毫秒和微秒
 */
void Dwt_Sys_Time_Update(void);

/**
 * @brief 获取系统时间线(秒)
 * @note 返回系统运行的总时间，单位为秒，精度到微秒
 * @return float 系统运行时间，单位为秒
 */
float Dwt_Get_Time_Line_S(void);

/**
 * @brief 获取系统时间线(毫秒)
 * @note 返回系统运行的总时间，单位为毫秒，精度到微秒
 * @return float 系统运行时间，单位为毫秒
 */
float Dwt_Get_Time_Line_Ms(void);

/**
 * @brief 获取系统时间线(微秒)
 * @note 返回系统运行的总时间，单位为微秒
 * @return uint32_t 系统运行时间，单位为微秒
 */
uint32_t Dwt_Get_Time_Line_Us(void);

/**
 * @brief 阻塞式延时函数(秒)
 * @note 使用DWT计数器实现精确延时，单位为秒,范围为1-255秒
 * @param delay_time 延时时间，单位为秒
 */
void Dwt_Delay_S(const uint8_t delay_time);

/**
 * @brief 阻塞式延时函数(毫秒)
 * @note 使用DWT计数器实现精确延时，单位为毫秒，范围为1-65535毫秒
 * @param delay_time 延时时间，单位为毫秒
 */
void Dwt_Delay_Ms(const uint16_t delay_time);

/**
 * @brief 阻塞延式时函数(微秒)
 * @note 使用DWT计数器实现精确延时，单位为微秒，范围为1-65535微秒
 * @param delay_time 延时时间，单位为微秒
 */
void Dwt_Delay_Us(const uint16_t delay_time);

#endif /* __BSP_DWT_H */