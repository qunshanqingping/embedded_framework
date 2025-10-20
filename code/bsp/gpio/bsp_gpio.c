#include "bsp_gpio.h"
#include "memory_management.h"
#include "stdlib.h"
#include "string.h"
#include "stdbool.h"
#include "plf_log.h"

/* 私有函数原型 -------------------------------------------------------------*/

/* 函数定义 ------------------------------------------------------------------*/

/**
 * @brief GPIO 注册参数检查函数
 * @param config GPIO 初始化配置结构体指针
 * @return true 参数合法
 * @return false 参数非法
 */
static bool Gpio_Register_Check(GpioInitConfig_s *config)
{
    if (config == NULL)
    {
        Log_Error("Gpio Register Failed: config is NULL");
        return false;
    }
    if (config->topic_name == NULL)
    {
        Log_Error("Gpio Register Failed: topic_name is NULL");
        return false;
    }
    if (config->port == NULL)
    {
        Log_Error("Gpio Register Failed: port is NULL");
        return false;
    }
    if (config->pin == 0)
    {
        Log_Error("Gpio Register Failed: pin is 0");
        return false;
    }
    return true;
}
GpioInstance_s * Gpio_Register(GpioInitConfig_s *config)
{
    if ((Gpio_Register_Check(config)) == false)
    {
        return NULL;
    }
    GpioInstance_s *instance = user_malloc(sizeof(GpioInstance_s));
    if (instance == NULL){
        Log_Error("Gpio Register Failed: instance Malloc Failed");
        return NULL;
    }
    memset(instance, 0, sizeof(GpioInstance_s));
    instance->topic_name = config->topic_name;
    instance->port = config->port;
    instance->pin = config->pin;
    instance->callback = config->callback;
    instance->parent_ptr = config->parent_ptr;
    return instance;
}

/**
 * @brief 设置 GPIO 引脚为高电平
 * @param instance GPIO 实例指针
 */
void Gpio_Set(GpioInstance_s *instance){
    HAL_GPIO_WritePin(instance->port,instance->pin,GPIO_PIN_SET);
}

/**
 * @brief 设置 GPIO 引脚为低电平
 * @param instance GPIO 实例指针
 */
void Gpio_Reset(GpioInstance_s *instance){
    HAL_GPIO_WritePin(instance->port,instance->pin,GPIO_PIN_RESET);
}