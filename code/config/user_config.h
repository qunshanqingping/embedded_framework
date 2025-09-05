/**
 *@file user_config.h
 *@brief 初始化配置文件
 *@version 1.0
 *@date 2025-08-27
 *@note 该文件用于存放用户的配置选项，如宏定义、常量；避免对 bsp 层文件的直接修改
 *@author Adonis_Jin
 */

#ifndef USER_CONFIG_H
#define USER_CONFIG_H


#include "stdlib.h"
#include <stdint.h>
#include "cmsis_os.h"
#include "memory.h"

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

#ifndef user_free
#ifdef _CMSIS_OS_H
#define user_free vPortFree
#else
#define user_malloc free
#endif
#endif

#include "stdbool.h"
/* 调试模式 */
#define DEBUG_MODE

/* CAN 初始化配置选项 */

// 选择 CAN 类型
// #define USER_CAN_FD
#define USER_CAN_STANDARD

// 选择 CAN 总线
#define USER_CAN1
#define USER_CAN2
#define USER_CAN3

//选择 can fifo 0 or 1
#ifdef USER_CAN1
#define USER_CAN1_FIFO_0
#define USER_CAN1_FIFO_1
#endif

#ifdef USER_CAN2
#define USER_CAN2_FIFO_0
#define USER_CAN2_FIFO_1
#endif

#ifdef USER_CAN3
#define USER_CAN3_FIFO_0
#define USER_CAN3_FIFO_1
#endif

// #define USER_CAN_FILTER_MASK_MODE
#define S
// // 检查是否出现定义冲突, 只允许一个 CAN 类型定义存在, 否则编译会自动报错
// #if defined(USER_CAN_FD) && defined(USER_CAN_STANDARD)
// #error "USER_CAN_FD 和 USER_CAN_STANDARD 不能同时定义！"
// #endif
// // 检查是否出现定义冲突, 标准 can 只支持 CAN1 和 CAN2
// #if defined(USER_CAN_STANDARD) &&defined(USER_CAN3)
// #error "USER_CAN_STANDARD 不支持CAN3！"
// #endif
//
// #if defined(USER_CAN_STANDARD) && !defined(USER_CAN1) && !defined(USER_CAN2)
// #error "USER_CAN_STANDARD 至少需要定义一个 CAN 总线！"
// #endif
// #if defined(USER_CAN_FD) && !defined(USER_CAN1) && !defined(USER_CAN2) && !defined(USER_CAN3)
// #error "USER_CAN_FD 至少需要定义一个 CAN 总线！"
// #endif
// /* 底盘初始化配置选项 */
// // 选择底盘类型
// #define USER_OMNI // 全向轮底盘
// // #define USER_MECANUM // 麦克纳姆底盘
// // #define USER_SWERVE // 舵轮底盘
//
// // 检查是否出现定义冲突, 只允许一个底盘类型定义存在, 否则编译会自动报错
// #if (defined(USER_OMNI) && defined(USER_MECANUM))||(defined(USER_OMNI) && defined(USER_SWERVE))||(defined(USER_MECANUM) && defined(USER_SWERVE))
// #error "USER_OMNI, USER_MECANUM 和 USER_SWERVE 只能定义一个！"
// #endif
#endif //USER_CONFIG_H
