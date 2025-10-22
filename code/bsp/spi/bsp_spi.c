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
/* 私有变量 -----------------------------------------------------------------*/

static SpiInstance_s *spi_instances[SPI_DEVICE_CNT]={NULL};
static uint8_t spi_idx = 0;
// /**
//  * 配合中断以及初始化
//  */
// static uint8_t idx = 0;
//
// /**
//  * @brief 用于判断当前 spi 是否正在传输, 防止多个模块同时使用一个 spi 总线
//  *        0-- 正在传输; 1-- 未传输
//  * @note 弃用，感觉不如直接读 spi 寄存器，有点多此一举，并且在调用 hal 库的 spi 函数时，函数本身就会先判断是否正在传输
//  */
// // uint8_t SPIDeviceOnGoing[SPI_DEVICE_CNT] = {1};
//
static bool Spi_Register_Check(SpiInitConfig_s *config)
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
SpiInstance_s* Spi_Register(SpiInitConfig_s* config){
    if (Spi_Register_Check(config) == false){
        return NULL;
    }
    SpiInstance_s* instance = user_malloc(sizeof(SpiInstance_s));
    if (instance == NULL){
        Log_Error("Spi_Register: %s Malloc Failed",config->topic_name);
        return NULL;
    }
    memset(instance, 0, sizeof(SpiInstance_s));
    instance->topic_name = config->topic_name;
    instance->mode = config->mode;
    instance->spi_handle = config->spi_handle;
    instance->cs_port = config->cs_port;
    instance->cs_pin = config->cs_pin;
    instance->timeout = config->timeout;
    spi_instances[spi_idx++] = instance;
    return instance;
}
// /**
//  * @file bsp_spi.c
//  * @brief SPI 实例注册函数
//  * @param config SPI 初始化配置结构体指针
//  * @return instance 指针 -- 注册成功   NULL-- 注册失败
//  * @note 删去了寻找SPI句柄的功能，因为传入一个未定义的 SPI 句柄过不了编译阶段的检查
//  */
// SpiInstance_s *Spi_Register(SpiInitConfig_s *config)
// {
//     if (config == NULL)
//     {
//         if (SPI_DEBUG_MODE)
//         {
//             while (1);
//         }
//         return NULL;
//     }
//     uint8_t slave_cnt = 0; // 记录 SPI 从设备数量
//     for (uint8_t i = 0; i < idx; i++)
//     {
//         if (spi_instances[i]->spi_handle.Instance== config->spi_handle.Instance )
//         {
//             slave_cnt++;
//         }
//     }
//     if (slave_cnt >= MX_SPI_BUS_SLAVE_CNT)
//     {
//         if (SPI_DEBUG_MODE)
//         {
//             while (1);
//         }
//         return NULL; // 超过最大从设备数量
//     }
//     // 分配内存空间
//     SpiInstance_s *instance = (SpiInstance_s *)pvPortMalloc(sizeof(SpiInstance_s));
//     if (instance == NULL)
//     {
//         vPortFree(instance);
//         if (SPI_DEBUG_MODE)
//         {
//             while (1);
//         }
//         return NULL;
//     }
//     memset(instance, 0, sizeof(SpiInstance_s));
//
//     instance->spi_handle = config->spi_handle; // 复制 SPI 句柄
//     instance->mode = config->mode; // 设置 SPI 模式
//     instance->cs_mode = config->cs_mode; // 设置片选模式
//     if (config->cs_mode == SPI_CS_ENABLE)
//     {
//         // 如果片选模式为使能，则设置片选引脚
//         instance->cs_pin = config->cs_port; // 设置片选端口
//         instance->cs_pin = config->cs_pin; // 设置片选引脚
//     }
//     instance->id = config->id;
//     spi_instances[idx] = instance;
//     idx++;
//     return instance;
// }
//
// /**
//  * @brief 读取 SPI 片选引脚状态
//  * @param spi_ins SPI 实例指针
//  * @note 仅出于美观和一致性考虑，对HAL_GPIO_ReadPin进行重写，实际使用中可以直接调用 HAL 库的函数
//  * @details 此函数用于读取 SPI 片选引脚的当前状态，并更新实例中的状态变量
//  */
// void Spi_Cs_ReadPinState(SpiInstance_s *spi_ins)
// {
//     assert_param(IS_GPIO_PIN(spi_ins->cs_pin));
//     if ((spi_ins->cs_pin->IDR & spi_ins->cs_pin) != 0x00U)
//     {
//         spi_ins->cs_pin_state = GPIO_PIN_SET;
//     }
//     else
//     {
//         spi_ins->cs_pin_state = GPIO_PIN_RESET;
//     }
// }
//
//
// /**
//  * @brief 拉高片选引脚
//  * @param spi_ins spi 实例指针
//  */
// void spi_cs_high(SpiInstance_s *spi_ins)
// {
//     HAL_GPIO_WritePin(spi_ins->cs_pin, spi_ins->cs_pin, GPIO_PIN_SET);
//     spi_ins->cs_pin_state = HAL_GPIO_ReadPin(spi_ins->cs_pin, spi_ins->cs_pin);
// }
//
// /**
//  * @brief 拉低片选引脚
//  * @param spi_ins spi 实例指针
//  */
// void spi_cs_low(SpiInstance_s *spi_ins)
// {
//     HAL_GPIO_WritePin(spi_ins->cs_pin, spi_ins->cs_pin, GPIO_PIN_RESET);
//     spi_ins->cs_pin_state = HAL_GPIO_ReadPin(spi_ins->cs_pin, spi_ins->cs_pin);
// }
//
//
// /**
//  * @brief SPI 发送数据函数
//  * @param spi_ins SPI 实例指针
//  * @param tx_data 发送数据指针
//  * @param tx_len 发送数据长度
//  * @return true-- 发送成功   false-- 发送失败
//  * @details 此函数用于通过 SPI 发送数据，支持阻塞、中断和 DMA 模式
//  *          status SPI 状态 返回值为 HAL_OK 表示发送成功，其他值表示发送失败
//  * @note 在使用此函数时，需要确保 SPI 实例已经正确初始化
//  * @todo 更详细的错误处理和调试信息
//  */
// bool Spi_Transmit(SpiInstance_s *spi_ins, uint8_t *tx_data, uint16_t tx_len)
// {
//     if (spi_ins == NULL || tx_data == NULL || tx_len == 0)
//     {
//         if (SPI_DEBUG_MODE)
//         {
//             while (1);
//         }
//         return false; // 参数错误
//     }
//     HAL_StatusTypeDef status;
//     switch (spi_ins->mode)
//     {
//     case SPI_BLOCKING_MODE:
//         status = HAL_SPI_Transmit(&spi_ins->spi_handle, tx_data, tx_len, SPI_TIMEOUT_MS);
//         if (status != HAL_OK)
//         {
//             if (SPI_DEBUG_MODE)
//             {
//                 while (1);
//             }
//             return false; // 发送失败
//         }
//         break;
//     case SPI_IT_MODE:
//         status = HAL_SPI_Transmit_IT(&spi_ins->spi_handle, tx_data, tx_len);
//         if (status != HAL_OK)
//         {
//             if (SPI_DEBUG_MODE)
//             {
//                 while (1);
//             }
//             return false; // 发送失败
//         }
//         break;
//     case SPI_DMA_MODE:
//         status = HAL_SPI_Transmit_DMA(&spi_ins->spi_handle, tx_data, tx_len);
//         if (status != HAL_OK)
//         {
//             if (SPI_DEBUG_MODE)
//             {
//                 while (1);
//             }
//             return false; // 发送失败
//         }
//         break;
//     default:
//         if (SPI_DEBUG_MODE)
//         {
//             while (1);
//         }
//         return false; // 发送失败
//         break;
//     }
//     return true; // 发送成功
// }
//
/**
 * @brief SPI 发送数据函数
 * @param instance SPI 实例指针
 * @param tx_data 发送数据指针
 * @param tx_len 发送数据长度
 */
void Spi_Transmit(SpiInstance_s* instance, uint8_t* tx_data, uint16_t tx_len){
    if (instance == NULL || tx_data == NULL || tx_len == 0){
        Log_Error("Spi_Transmit Fail : Invalid Parameter");
        return ;
    }
    switch (instance->mode){
    case BLOCK_MODE: {
        HAL_SPI_Transmit(instance->spi_handle, tx_data, tx_len, SPI_TIMEOUT_MS);
        break;
    }
    case DMA_MODE: {
        HAL_SPI_Transmit_DMA(instance->spi_handle, tx_data, tx_len);
        break;
    }
    case IT_MODE: {
        HAL_SPI_Transmit_IT(instance->spi_handle, tx_data, tx_len);
        break;
    }
    default: {
        Log_Error("Spi_Transmit Fail : Invalid Transfer Mode");
    }
    }
}

// /**
//  * @brief SPI 接收数据函数
//  * @param spi_ins SPI 实例指针
//  * @param rx_data 接收数据指针
//  * @param rx_len 接收数据长度
//  * @return true-- 接收成功   false-- 接收失败
//  * @details 此函数用于通过 SPI 接收数据，支持阻塞、中断和 DMA 模式
//  *          status SPI 状态 返回值为 HAL_OK 表示接收成功，其他值表示接收失败
//  * @note 在使用此函数时，需要确保 SPI 实例已经正确初始化
//  * @todo 更详细的错误处理和调试信息
//  */
// bool Spi_Receive(SpiInstance_s *spi_ins, uint8_t *rx_data, uint16_t rx_len){
//     if (spi_ins == NULL || rx_data == NULL || rx_len == 0){
//         if (SPI_DEBUG_MODE){
//             while (1);
//         }
//         return false; // 参数错误
//     }
//     HAL_StatusTypeDef status;
//     switch (spi_ins->mode)
//     {
//     case SPI_BLOCKING_MODE:
//         status = HAL_SPI_Receive(&spi_ins->spi_handle, rx_data, rx_len, SPI_TIMEOUT_MS);
//         if (status != HAL_OK)
//         {
//             if (SPI_DEBUG_MODE){
//                 while (1);
//             }
//             return false; // 接收失败
//         }
//         break;
//     case SPI_IT_MODE:
//         status = HAL_SPI_Receive_IT(&spi_ins->spi_handle, rx_data, rx_len);
//         if (status != HAL_OK)
//         {
//             if (SPI_DEBUG_MODE){
//                 while (1);
//             }
//             return false; // 接收失败
//         }
//         break;
//     case SPI_DMA_MODE:
//         status = HAL_SPI_Receive_DMA(&spi_ins->spi_handle, rx_data, rx_len);
//         if (status != HAL_OK)
//         {
//             if (SPI_DEBUG_MODE){
//                 while (1);
//             }
//             return false;
//         }
//         break;
//     default:
//         if (SPI_DEBUG_MODE)
//         {
//             while (1);
//         }
//         return false; // 接收模式错误
//         break;
//     }
//     return true; // 接收成功
// }
//
// /**
//  * @brief SPI 发送接收数据函数
//  * @param spi_ins SPI 实例指针
//  * @param tx_data 发送数据指针
//  * @param rx_data 接收数据指针
//  * @param len 数据长度
//  * @return true-- 发送接收成功   false-- 发送接收失败
//  * @details 此函数用于通过 SPI 同时发送和接收数据，支持阻塞、中断和 DMA 模式
//  *          status SPI 状态 返回值为 HAL_OK 表示发送接收成功，其他值表示发送接收失败
//  * @note 在使用此函数时，需要确保 SPI 实例已经正确初始化
//  * @todo 更详细的错误处理和调试信息
//  */
// bool Spi_TransmitReceive(SpiInstance_s *spi_ins, uint8_t *tx_data, uint8_t *rx_data, uint16_t len)
// {
//     if (spi_ins == NULL || tx_data == NULL || rx_data == NULL || len == 0)
//     {
//         if (SPI_DEBUG_MODE)
//         {
//             while (1);
//         }
//         return false; // 参数错误
//     }
//     HAL_StatusTypeDef status;
//     switch (spi_ins->mode)
//     {
//     case SPI_BLOCKING_MODE:
//         status = HAL_SPI_TransmitReceive(&spi_ins->spi_handle, tx_data, rx_data, len, SPI_TIMEOUT_MS);
//         if (status != HAL_OK)
//         {
//             if (SPI_DEBUG_MODE)
//             {
//                 while (1);
//             }
//             return false; // 发送接收失败
//         }
//         break;
//     case SPI_IT_MODE:
//         status = HAL_SPI_TransmitReceive_IT(&spi_ins->spi_handle, tx_data, rx_data, len);
//         if (status != HAL_OK)
//         {
//             if (SPI_DEBUG_MODE)
//             {
//                 while (1);
//             }
//             return false; // 发送接收失败
//         }
//         break;
//     case SPI_DMA_MODE:
//         status = HAL_SPI_TransmitReceive_DMA(&spi_ins->spi_handle, tx_data, rx_data, len);
//         if (status != HAL_OK)
//         {
//             if (SPI_DEBUG_MODE)
//             {
//                 while (1);
//             }
//             return false; // 发送接收失败
//         }
//         break;
//     default:
//         if (SPI_DEBUG_MODE)
//         {
//             while (1);
//         }
//         return false; // 发送接收模式错误
//         break;
//     }
//     return true; // 发送接收成功
// }
//
/**
 * @brief SPI 发送接收数据函数
 * @param instance SPI 实例指针
 * @param tx_data 发送数据指针
 * @param rx_data 接收数据指针
 * @param len 数据长度
 * @param timeout 超时时间
 */
void Spi_TransmitReceive(SpiInstance_s * instance, uint8_t* tx_data, uint8_t* rx_data, uint16_t len,uint16_t timeout){
    if (instance == NULL || tx_data == NULL || rx_data == NULL || len == 0){
        Log_Error("Spi_TransmitReceive Fail : Invalid Parameter");
        return ;
    }
    switch (instance->mode){
        case BLOCK_MODE: {
            HAL_SPI_TransmitReceive(instance->spi_handle, tx_data, rx_data, len,timeout);
            break;
        }
        case DMA_MODE: {
            HAL_SPI_TransmitReceive_DMA(instance->spi_handle, tx_data, rx_data, len);
            break;
        }
        case IT_MODE:{
            HAL_SPI_TransmitReceive_IT(instance->spi_handle, tx_data, rx_data, len);
            break;
        }
        default: {
            Log_Error("Spi_TransmitReceive Fail : Invalid Transfer Mode");
        }
    }
}
// /**
//  * @brief SPI 接收完成回调函数
//  * @param hspi SPI 句柄
//  * @details 此函数用于在 SPI 接收完成后调用，遍历所有 SPI 实例并调用对应的回调函数
//  * @note 对cs引脚的操作在用户自定义回调函数中进行
//  */
// static void Bsp_Spi_RxCpltCallback(SPI_HandleTypeDef *hspi)
// {
//     // 遍历所有 SPI 实例，找到匹配的实例并调用回调函数
//     for (uint8_t i = 0; i < idx; i++)
//     {
//         if (spi_instances[i] != NULL && spi_instances[i]->spi_handle.Instance == hspi->Instance)
//         {
//             if (spi_instances[i]->spi_module_callback != NULL)
//             {
//                 spi_instances[i]->spi_module_callback(spi_instances[i]);
//             }
//             break;
//         }
//     }
// }
// /**
//  * @brief 对week HAL_SPI_RxCpltCallback 函数的实现
//  * @param hspi SPI 句柄
//  * @details 此函数用于处理 SPI 中断
//  */
// void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
// {
//     Bsp_Spi_RxCpltCallback(hspi);
// }
//
// /**
//  * @brief 对week HAL_SPI_TxRxCpltCallback 函数的实现
//  * @param hspi SPI 句柄
//  * @details 此函数用于处理 SPI 中断
//  */
// void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
// {
//     Bsp_Spi_RxCpltCallback(hspi);
// }
