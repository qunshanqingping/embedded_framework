#ifndef BSP_GPIO_H
#define BSP_GPIO_H
#include "gpio.h"

typedef struct GpioInstance{
    char *topic_name;
    GPIO_TypeDef* port;
    uint16_t pin;
    void(*callback)(struct GpioInstance*);
    void* parent_ptr;
}GpioInstance_s;

typedef struct {
    char *topic_name;
    GPIO_TypeDef* port;
    uint16_t pin;
    void(*callback)(struct GpioInstance*);
    void *parent_ptr;
}GpioInitConfig_s;


/**
 * @brief GPIO 注册函数
 * @param config GPIO 初始化配置结构体指针
 * @return GpioInstance_s* GPIO 实例指针
 */
GpioInstance_s * Gpio_Register(GpioInitConfig_s *config);

/**
 * @brief 设置 GPIO 引脚为高电平
 * @param instance GPIO 实例指针
 */
void Gpio_Set(GpioInstance_s *instance);

/**
 * @brief 设置 GPIO 引脚为低电平
 * @param instance GPIO 实例指针
 */
void Gpio_Reset(GpioInstance_s *instance);

#endif //BSP_GPIO_H