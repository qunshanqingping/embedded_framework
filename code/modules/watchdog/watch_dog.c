#include "watch_dog.h"
#include "plf_log.h"
#include "memory_management.h"

#include <string.h>
#include <stdint.h>

/* 看门狗实例索引 */
static uint8_t watch_dog_idx = 0;
/* 看门狗实例数组 */
static WatchDogInstance_s* watchdog_instance[WATCH_DOG_MAX_REGISTER_CNT];
/**
 * @brief 注册并初始化看门狗实例
 * @param config 看门狗的初始化配置结构体指针
 * @return 成功返回看门狗实例指针，失败返回NULL
 */
WatchDogInstance_s* WatchDog_Register(const WatchDogInitConfig_s* config){
    if (config == NULL){
        Log_Error("WatchDog Register Config Is NULL");
        return NULL;
    }
    if (config->topic_name == NULL){
        Log_Error("WatchDog Register Topic Name Is NULL");
        return NULL;
    }
    if (config->watchdog_callback == NULL){
        Log_Error("WatchDog %s Callback Is NULL", config->topic_name);
        return NULL;
    }
    if (watch_dog_idx >= WATCH_DOG_MAX_REGISTER_CNT){
        Log_Error("WatchDog %s Max Register Count Reached", config->topic_name);
        return NULL;
    }
    WatchDogInstance_s* instance = user_malloc(sizeof(WatchDogInstance_s));
    if (instance == NULL){
        Log_Error("WatchDog %s Instance Malloc Failed", config->topic_name);
        return NULL;
    }
    memset(instance, 0, sizeof(WatchDogInstance_s));
    instance->topic_name = config->topic_name;
    instance->watchdog_callback = config->watchdog_callback;
    instance->parent_ptr = config->parent_ptr;
    watchdog_instance[watch_dog_idx++] = instance;
    Log_Passing("WatchDog %s Register Success", config->topic_name);
    return instance;
}

/**
 * @brief 看门狗回调函数
 * @note 该函数会遍历所有注册的看门狗实例，并调用它们的回调函数
 */
void WatchDog_Callback(void){
    for (uint8_t i = 0; i < watch_dog_idx; i++){
        if (watchdog_instance[i]->watchdog_callback != NULL){
            watchdog_instance[i]->watchdog_callback(watchdog_instance[i]);
        }
    }
}
