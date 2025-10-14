#ifndef WATCH_DOG_H
#define WATCH_DOG_H
#include <stdint.h>

#define WATCH_DOG_MAX_REGISTER_CNT 150

typedef struct WatchDogInstance_s{
    char* topic_name; // 实例名称
    void (*watchdog_callback)(struct WatchDogInstance_s*); // 看门狗的回调函数
    void* parent_ptr; // 使用看门狗的父模块指针
} WatchDogInstance_s;

typedef struct{
    char* topic_name; // 实例名称
    void (*watchdog_callback)(struct WatchDogInstance_s*); // 看门狗的回调函数
    void* parent_ptr; // 使用看门狗的父模块指针
} WatchDogInitConfig_s;

/**
 * @brief 注册并初始化看门狗实例
 * @param config 看门狗的初始化配置结构体指针
 * @return 成功返回看门狗实例指针，失败返回NULL
 */
WatchDogInstance_s* WatchDog_Register(const WatchDogInitConfig_s* config);

/**
 * @brief 看门狗回调函数
 * @note 该函数会遍历所有注册的看门狗实例，并调用它们的回调函数
 */
void WatchDog_Callback(void);
#endif //WATCH_DOG_H
