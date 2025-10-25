#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdbool.h>
#include <stdint.h>
/* 开发配置 */
#define BARE_METAL    // 裸机模式
// #define FREERTOS      // FreeRTOS模式

/* 底盘配置 */
#define MECANUM // 麦克纳姆轮底盘
// #define OMNI    // 全向轮底盘
// #define BALANCE // 平衡底盘
// #define STEER   // 舵轮底盘

/* 遥控器配置 */
#define DJI_DT7    // 大疆遥控器
//#define FLY_SKY_I6X // 富斯I6X遥控器

/* CAN配置 */

/* 选择 CAN 类型 */
#define USER_CAN_FD
// #define USER_CAN_STD
/* 选择 CAN 路数 */
#define USER_CAN1
#define USER_CAN2
#define USER_CAN3
/* 选择 CAN1 使用的 FIFO */
// #define USER_CAN1_FIFO_0
#define USER_CAN1_FIFO_1
/* 选择 CAN2 使用的 FIFO */
#define USER_CAN2_FIFO_0
// #define USER_CAN2_FIFO_1
/* 选择 CAN3 使用的 FIFO */
#define USER_CAN3_FIFO_0
// #define USER_CAN3_FIFO_1
/* 选择 CAN 过滤器模式 */
#define USER_CAN_FILTER_MASK_MODE /*H7目前只支持掩码，因为我懒 */
// #define USER_CAN_FILTER_LIST_MODE


/* 配置检查 */

/* 开发配置 */
#if (defined(BARE_METAL) && defined(FREERTOS))||( !defined(BARE_METAL) && !defined(FREERTOS) )
#error "只能选择一种开发模式: BARE_METAL 或 FREERTOS"
#endif
/* 底盘配置 */
#if (defined(MECANUM) && defined(OMNI)) || (defined(MECANUM) && defined(BALANCE)) || (defined(MECANUM) && defined(STEER)) || (defined(OMNI) && defined(BALANCE)) || (defined(OMNI) && defined(STEER)) || (defined(BALANCE) && defined(STEER))
#error "只能选择一种底盘类型: MECANUM, OMNI, BALANCE 或 STEER"
#endif
/* 遥控器配置 */
#if (defined(DJI_DT7) && defined(FLY_SKY_I6X))
#error "只能选择一种遥控器类型: DJI_DT7 或 FLY_SKY_I6X"
#endif
/* CAN配置 */
#if (defined(USER_CAN_FD) && defined(USER_CAN_STD))
#error "只能选择一种CAN类型: USER_CAN_FD 或 USER_CAN_STD"
#endif
#if defined (USER_CAN_STD)
#if defined (USER_CAN3)
#error "CAN STD 模式下不支持 CAN3"
#endif
#if defined (USER_CAN2) && !defined (USER_CAN1)
#error "CAN_STD 模式 CAN2 是 CAN1 的从机，必须启用 CAN1"
#endif
#endif
/* can FIFO 配置 */
#if !defined(USER_CAN1) && (defined(USER_CAN1_FIFO_0) || defined(USER_CAN1_FIFO_1))
#error "未启用 CAN1，但配置了 CAN1 的 FIFO"
#endif
#if !defined(USER_CAN2) && (defined(USER_CAN2_FIFO_0) || defined(USER_CAN2_FIFO_1))
#error "未启用 CAN2，但配置了 CAN2 的 FIFO"
#endif
#if !defined(USER_CAN3) && (defined(USER_CAN3_FIFO_0) || defined(USER_CAN3_FIFO_1))
#error "未启用 CAN3，但配置了 CAN3 的 FIFO"
#endif
#endif //ROBOT_CONFIG_H