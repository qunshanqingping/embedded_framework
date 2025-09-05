/**
 *@file bsp_log.h
 *@author Adonis Jin
 *@brief 通过J_link的RTT功能实现日志输出
 *@date 2025-06-03
 *@version 1.1
 *@note 实现基本功能
 *@date 2025-8-26
 *@version 1.1
 *@note 删去时间戳功能，意义不大，规范命名
 */


#ifndef BSP_LOG_H
#define BSP_LOG_H

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
/* Private includes -----------------------------------------------------------*/
#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
/* Private types -------------------------------------------------------------*/


/* Private defines -----------------------------------------------------------*/
#define BUFFER_INDEX 0

/* Private macro -------------------------------------------------------------*/

/**
 * @brief 日志功能原型,供下面的函数使用
 *
 */
#define LOG_PROTO(type, color, format, ...)                       \
        SEGGER_RTT_printf(BUFFER_INDEX, "  %s%s" format "\r\n%s", \
                          color,                                  \
                          type,                                   \
                          ##__VA_ARGS__,                          \
                          RTT_CTRL_RESET)
/**
 * @brief 清屏
 */
#define Log_Clear() SEGGER_RTT_WriteString(0, "  " RTT_CTRL_CLEAR)

/**
 *@brief 无颜色日志输出
 * @param format 输出内容
 */
#define Log(format, ...) LOG_PROTO("", "", format, ##__VA_ARGS__)

/**
 * @brief 信号输出 黑色
 * @param format 输出内容
 */
#define Log_Information(format, ...) LOG_PROTO("I:", RTT_CTRL_TEXT_BRIGHT_BLACK, format, ##__VA_ARGS__)

/**
 * @brief 信号输出 绿色
 * @param format 输出内容
 */
#define Log_Passing(format, ...) LOG_PROTO("P:", RTT_CTRL_TEXT_BRIGHT_GREEN, format, ##__VA_ARGS__)

/**
 * @brief 信号输出 蓝色
 * @param format 输出内容
 */
#define Log_Debug(format, ...) LOG_PROTO("D:", RTT_CTRL_TEXT_BRIGHT_BLUE, format, ##__VA_ARGS__)
/**
 * @brief 警告输出 黄色
 * @param format 输出内容
 */
#define Log_Warning(format, ...) LOG_PROTO("W:", RTT_CTRL_TEXT_BRIGHT_YELLOW, format, ##__VA_ARGS__)

/**
 * @brief 错误输出 红色
 * @param format 输出内容
 */
#define Log_Error(format, ...) LOG_PROTO("E:", RTT_CTRL_TEXT_BRIGHT_RED, format, ##__VA_ARGS__)


/* Exported variables ---------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief 日志系统初始化，这里为了统一bsp层，重新封装
 */
void Log_Init();

inline void PrintS_B_Error() {
    // 使用Log_Error和0输出"S_B"字符画
    Log_Error(" 000000   000000 ");
    Log_Error("00        00   00 ");
    Log_Error("00        00   00 ");
    Log_Error(" 000000   000000 ");
    Log_Error("      00  00   00");
    Log_Error("      00  00   00");
    Log_Error(" 000000   000000 ");
}
#endif /* BSP_LOG_H */
