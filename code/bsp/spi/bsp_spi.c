/**
 * @file bsp_spi.c
 * @author Adonis Jin
 * @date 2025-08-08
 * @version 1.0.0
 * @note 对CS引脚的操作需要在调用 SPI 传输函数前后手动进行
 *       本文件只提供Spi_Cs_ReadPinState函数用于读取CS引脚状态
 *                 Spi_cs_high函数用于拉高CS引脚
 *                 Spi_cs_low函数用于拉低CS引脚
 */

/* 包含文件 ------------------------------------------------------------------*/
#include "bsp_spi.h"
#include "memory.h"
#include "stdlib.h"
#include "FreeRTOS.h"
#include "plf_log.h"
#include "memory_management.h"
#include "robot_config.h"
/* 私有变量 -----------------------------------------------------------------*/

static SpiInstance_s* spi_instances[SPI_DEVICE_CNT] = {NULL};
static uint8_t spi_idx = 0;


/**
 * @brief 选择spi端口指针
 * @param spi_handle_number spi端口号
 * @return 对应端口的指针
 */
SPI_HandleTypeDef* Spi_handle_Select(uint8_t spi_handle_number)
{
    if (spi_handle_number == 0)
    {
        Log_Error("Spi_handle_Select : Invalid SPI handle number: zero");
        return NULL;
    }
    switch (spi_handle_number)
    {
    case 1:
        {
#if defined(USER_SPI1)
            return &hspi1;
#endif
        }
    case 2:
        {
#if defined (USER_SPI2)
            return &hspi2;
#endif
        }
    case 3:
        {
#if defined (USER_SPI3)
            return &hspi3;
#endif
        }
    case 4:
        {
#if defined (USER_SPI4)
            return &hspi4;
#endif
        }
    case 5:
        {
#if defined (USER_SPI5)
            return &hspi5;
#endif
        }
    default:
        {
            Log_Error("Spi_handle_Select : Invalid SPI handle number");
            return NULL;
        }
    }
}

/**
 * @brief SPI 注册检查函数
 * @param config SPI 初始化配置结构体指针
 * @return true-- 检查通过   false-- 检查失败
 */
static bool Spi_Register_Check(SpiInitConfig_s* config)
{
    if (config == NULL)
    {
        Log_Error("Spi_Register_Check: config is NULL");
        return false;
    }
    if (config->topic_name == NULL)
    {
        Log_Error("Spi_Register_Check: topic_name is NULL");
        return false;
    }
    return true;
}

/**
 * @file bsp_spi.c
 * @brief SPI 实例注册函数
 * @param config SPI 初始化配置结构体指针
 * @return instance 指针 -- 注册成功   NULL-- 注册失败
 */
SpiInstance_s* Spi_Register(SpiInitConfig_s* config)
{
    if (Spi_Register_Check(config) == false)
    {
        return NULL;
    }
    SpiInstance_s* instance = user_malloc(sizeof(SpiInstance_s));
    if (instance == NULL)
    {
        Log_Error("Spi_Register: %s Malloc Failed", config->topic_name);
        return NULL;
    }
    memset(instance, 0, sizeof(SpiInstance_s));
    instance->topic_name = config->topic_name;
    instance->spi_handle = Spi_handle_Select(config->spi_handle_number);
    instance->mode = config->mode;
    instance->cs_port = config->cs_port;
    instance->cs_pin = config->cs_pin;
    instance->timeout = config->timeout;
    spi_instances[spi_idx++] = instance;
    return instance;
}

/**
 * @brief SPI 发送数据函数
 * @param instance SPI 实例指针
 * @param tx_data 发送数据指针
 * @param tx_len 发送数据长度
 */
void Spi_Transmit(SpiInstance_s* instance, uint8_t* tx_data, uint16_t tx_len)
{
    if (instance == NULL || tx_data == NULL || tx_len == 0)
    {
        Log_Error("Spi_Transmit Fail : Invalid Parameter");
        return;
    }
    if (instance->cs_port != NULL)
    {
        HAL_GPIO_WritePin(instance->cs_port, instance->cs_pin, GPIO_PIN_RESET);
    }
    switch (instance->mode)
    {
    case BLOCK_MODE:
        {
            HAL_SPI_Transmit(instance->spi_handle, tx_data, tx_len, SPI_TIMEOUT_MS);
            break;
        }
    case DMA_MODE:
        {
            HAL_SPI_Transmit_DMA(instance->spi_handle, tx_data, tx_len);
            break;
        }
    case IT_MODE:
        {
            HAL_SPI_Transmit_IT(instance->spi_handle, tx_data, tx_len);
            break;
        }
    default:
        {
            Log_Error("Spi_Transmit Fail : Invalid Transfer Mode");
        }
    }
    if (instance->cs_port != NULL)
    {
        HAL_GPIO_WritePin(instance->cs_port, instance->cs_pin, GPIO_PIN_SET);
    }
}

/**
 * @brief SPI 发送接收数据函数
 * @param instance SPI 实例指针
 * @param tx_data 发送数据指针
 * @param rx_data 接收数据指针
 * @param len 数据长度
 * @param timeout 超时时间
 */
void Spi_TransmitReceive(SpiInstance_s* instance, uint8_t* tx_data, uint8_t* rx_data, uint16_t len, uint16_t timeout)
{
    if (instance == NULL || tx_data == NULL || rx_data == NULL || len == 0)
    {
        Log_Error("Spi_TransmitReceive : Invalid Parameter");
        return;
    }
    if (instance->cs_port != NULL)
    {
        HAL_GPIO_WritePin(instance->cs_port, instance->cs_pin, GPIO_PIN_RESET);
    }
    switch (instance->mode)
    {
    case BLOCK_MODE:
        {
            HAL_SPI_TransmitReceive(instance->spi_handle, tx_data, rx_data, len, timeout);
            break;
        }
    case DMA_MODE:
        {
            HAL_SPI_TransmitReceive_DMA(instance->spi_handle, tx_data, rx_data, len);
            break;
        }
    case IT_MODE:
        {
            HAL_SPI_TransmitReceive_IT(instance->spi_handle, tx_data, rx_data, len);
            break;
        }
    default:
        {
            Log_Error("Spi_TransmitReceive Fail : Invalid Transfer Mode");
        }
    }
    if (instance->cs_port != NULL)
    {
        HAL_GPIO_WritePin(instance->cs_port, instance->cs_pin, GPIO_PIN_SET);
    }
}
