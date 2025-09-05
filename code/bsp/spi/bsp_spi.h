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

/* 宏定义 -------------------------------------------------------------------*/

/**
 * @brief SPI 调试模式，用于解决配置与 cubemx 配置之间的冲突
 * @param 0 发布模式
 * @note 发布模式将继续按照 cubemx 的配置进行配置
 * @param 1 调试模式将停止配置并终止程序
 */
#define SPI_DEBUG_MODE 0 // 0: 发布模式, 1: 调试模式

/**
 * @brief 根据不同开发板的 SPI 设备数量
 * @todo 不再局限于 DJI C 型开发板和达妙 MC02 开发板，而是根据实际工程需求
 */
#ifdef __STM32F407xx_HAL_H
/**
 * @brief DJI C 型开发板配备了两个 SPI 接口，
 * 分别连接到 BMI088 和扩展 IO，并从 8 针插座引出
 */
#define SPI_DEVICE_CNT 2
#endif
#ifdef STM32H7xx_HAL_H
/**
 * @brief 达妙 MC02 开发板配备了三个 SPI 接口，
 * 连接到 BMI088、WS2812B 和扩展 IO，并从 16 针插座引出
 */
#define SPI_DEVICE_CNT 3
#endif

/**
 * @brief 支持的 SPI 从设备最大数量
 * @note 此值可根据实际项目需求调整
 */
#define MX_SPI_BUS_SLAVE_CNT 4

/**
 * @brief SPI 超时时间，单位为毫秒
 * @note 此值可根据实际项目需求调整
 */
#define SPI_TIMEOUT_MS 100
/* 类型定义 ------------------------------------------------------------------*/

/**
 * @brief SPI 模式枚举
 * @details 此枚举定义了 SPI 通信的不同操作模式
 */
typedef enum
{
    SPI_BLOCKING_MODE = 0, //!< 阻塞模式，程序将等待直到 SPI 传输完成
    SPI_IT_MODE = 1,       //!< 中断模式，程序不会等待 SPI 传输完成，而是使用中断来通知完成
    SPI_DMA_MODE = 2,      //!< DMA 模式，程序不会等待 SPI 传输完成，而是使用 DMA 来传输数据
}SpiMode_e;

/**
 * @brief SPI 片选模式枚举
 * @details 此枚举定义了 SPI 片选信号的使能和禁用状态
 * @note 该枚举定义是为了兼容点对点，不存在 cs 引脚的设备
 */
typedef enum
{
    SPI_CS_ENABLE = 0,  //!< 需要 SPI 片选功能
    SPI_CS_DISABLE = 1, //!< 不需要 SPI 片选功能
}SpiCsMode_e;

#pragma pack(1)
/**
 * @brief SPI 实例化结构体
 */
typedef struct _SpiInstance_s
{
    SPI_HandleTypeDef spi_handle;                         //!< SPI 实例的句柄
    SpiMode_e mode;                                       //!< SPI 操作模式（阻塞、中断或 DMA）
    SpiCsMode_e cs_mode;                                  //!< SPI 片选模式（使能或禁用）

    GPIO_TypeDef *cs_port;                                //!< SPI 片选引脚所在的 GPIO 端口
    uint16_t cs_pin;                                      //!< SPI 片选引脚的 GPIO 引脚号
    GPIO_PinState cs_pin_state;                           //!< SPI 片选引脚的当前状态（高或低）

    void* (*spi_module_callback)(struct _SpiInstance_s*); //!< SPI 数据接收回调函数，用于处理接收到的数据
    void* id;                                             //!< 使用此 SPI 实例的父模块指针

}SpiInstance_s;

/**
 * @brief SPI 配置结构体
 */
typedef struct
{
    SPI_HandleTypeDef spi_handle;                          //!< SPI 实例的句柄
    SpiMode_e mode;                                        //!< SPI 操作模式（阻塞、中断或 DMA）
    SpiCsMode_e cs_mode;                                   //!< SPI 片选模式（使能或禁用）

    GPIO_TypeDef *cs_port;                                 //!< SPI 片选引脚所在的 GPIO 端口
    uint16_t cs_pin;                                       //!< SPI 片选引脚的 GPIO 引脚号

    void* (*spi_module_callback)(struct _SpiInstance_s*);  //!< SPI 数据接收回调函数，用于处理接收到的数据
    void* id;                                              //!< 使用此 SPI 实例的父模块指针
}SpiInitConfig_s;
#pragma pack()

/* 函数声明 ---------------------------------------------------------------*/
/**
 * @brief SPI 注册函数
 * @param spi_config SPI 配置结构体指针
 * @return true-- 初始化成功   false-- 初始化失败
 * @details 此函数用于初始化 SPI 实例，配置 SPI 参数和回调函数
 */
SpiInstance_s *Spi_Register(SpiInitConfig_s *spi_config);

/**
 * @brief 读取 SPI 片选引脚状态
 * @param spi_ins SPI 实例指针
 * @note 仅出于美观和一致性考虑，对HAL_GPIO_ReadPin进行重写，实际使用中可以直接调用 HAL 库的函数
 * @details 此函数用于读取 SPI 片选引脚的当前状态，并更新实例中的状态变量
 */
void Spi_Cs_ReadPinState(SpiInstance_s *spi_ins);

/**
 * @brief 拉高片选引脚
 * @param spi_ins spi 实例指针
 */
void spi_cs_high(SpiInstance_s *spi_ins);

/**
 * @brief 拉低片选引脚
 * @param spi_ins spi 实例指针
 */
void spi_cs_low(SpiInstance_s *spi_ins);

/**
 * @brief SPI 发送数据函数
 * @param spi_ins SPI 实例指针
 * @param tx_data 发送数据指针
 * @param tx_len 发送数据长度
 * @return true-- 发送成功   false-- 发送失败
 * @details 此函数用于通过 SPI 发送数据，支持阻塞、中断和 DMA 模式
 *          status SPI 状态 返回值为 HAL_OK 表示发送成功，其他值表示发送失败
 * @note 在使用此函数时，需要确保 SPI 实例已经正确初始化
 * @todo 更详细的错误处理和调试信息
 */
bool Spi_Transmit(SpiInstance_s *spi_ins, uint8_t *tx_data, uint16_t tx_len);

/**
 * @brief SPI 接收数据函数
 * @param spi_ins SPI 实例指针
 * @param rx_data 接收数据指针
 * @param rx_len 接收数据长度
 * @return true-- 接收成功   false-- 接收失败
 * @details 此函数用于通过 SPI 接收数据，支持阻塞、中断和 DMA 模式
 *          status SPI 状态 返回值为 HAL_OK 表示接收成功，其他值表示接收失败
 * @note 在使用此函数时，需要确保 SPI 实例已经正确初始化
 * @todo 更详细的错误处理和调试信息
 */
bool Spi_Receive(SpiInstance_s *spi_ins, uint8_t *rx_data, uint16_t rx_len);

/**
 * @brief SPI 发送接收数据函数
 * @param spi_ins SPI 实例指针
 * @param tx_data 发送数据指针
 * @param rx_data 接收数据指针
 * @param len 数据长度
 * @return true-- 发送接收成功   false-- 发送接收失败
 * @details 此函数用于通过 SPI 同时发送和接收数据，支持阻塞、中断和 DMA 模式
 *          status SPI 状态 返回值为 HAL_OK 表示发送接收成功，其他值表示发送接收失败
 * @note 在使用此函数时，需要确保 SPI 实例已经正确初始化
 * @todo 更详细的错误处理和调试信息
 */
bool Spi_TransmitReceive(SpiInstance_s *spi_ins, uint8_t *tx_data, uint8_t *rx_data, uint16_t len);

#endif //BSP_SPI_H