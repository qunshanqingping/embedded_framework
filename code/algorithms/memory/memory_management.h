#ifndef MEMORY_MANAGEMENT_H
#define MEMORY_MANAGEMENT_H
#include "robot_config.h"
#if defined FREERTOS
#define user_malloc pvPortMalloc
#endif

#if defined BARE_METAL
#define user_malloc malloc
#endif
#endif //MEMORY_MANAGEMENT_H