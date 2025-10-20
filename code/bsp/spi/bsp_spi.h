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

#ifndef BSP_SPI_H
#define BSP_SPI_H

/* 包含文件 ------------------------------------------------------------------*/

#include "spi.h"
#include "stdbool.h"
#include "bsp_typedef.h"
#include "bsp_gpio.h"

#define  SPI_TIMEOUT_MS 1000

typedef struct SpiInstance_s{
    TransferMode_e mode;
    SPI_HandleTypeDef* spi_handle;
    GpioInstance_s* cs_pin;
}SpiInstance_s;

/**
 * @brief SPI 发送数据函数
 * @param instance SPI 实例指针
 * @param tx_data 发送数据指针
 * @param tx_len 发送数据长度
 */
void Spi_Transmit(SpiInstance_s* instance, uint8_t* tx_data, uint16_t tx_len);
#endif //BSP_SPI_H