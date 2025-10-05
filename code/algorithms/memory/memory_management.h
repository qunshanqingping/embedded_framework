/**
* @file memory_management.h
* @author Adonis Jin
* @brief 内存管理头文件，定义了跨平台的内存分配接口
* @version 1.0
* @date 2024-06-20
*/

#ifndef MEMORY_MANAGEMENT_H
#define MEMORY_MANAGEMENT_H

#include "robot_config.h"

#if defined FREERTOS
#include "cmsis_os.h"
// FreeRTOS 平台下，user_malloc 映射为 pvPortMalloc
#define user_malloc pvPortMalloc
#endif

#if defined BARE_METAL
// 裸机平台下，user_malloc 映射为标准库 malloc
#define user_malloc malloc
#endif

#endif //MEMORY_MANAGEMENT_H