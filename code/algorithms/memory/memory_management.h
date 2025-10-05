/**
 * @file memory_management.h
 * @author Adonis Jin
 * @brief 内存管理头文件，定义了跨平台的内存分配接口
 * @version 1.0
 * @date 2024-06-20
 *
 * 该文件为内存管理模块，统一了不同平台下的内存分配方式。
 * 支持 FreeRTOS 和裸机（Bare Metal）两种平台。
 *
 * 使用方法：
 *   - 在需要动态内存分配的地方，直接调用 user_malloc。
 *   - user_malloc 会根据平台自动映射到对应的分配函数。
 */

#ifndef MEMORY_MANAGEMENT_H
#define MEMORY_MANAGEMENT_H

#include "robot_config.h"

#if defined FREERTOS
#include "cmsis_os.h"
/**
 * @brief FreeRTOS 平台下的内存分配接口
 * user_malloc 映射为 pvPortMalloc。
 */
#define user_malloc pvPortMalloc
#define user_free   vPortFree
#endif

#if defined BARE_METAL
/**
 * @brief 裸机平台下的内存分配接口
 *
 * user_malloc 映射为标准库 malloc。
 */
#define user_malloc malloc
#define user_free   free
#endif

#endif // MEMORY_MANAGEMENT_H
