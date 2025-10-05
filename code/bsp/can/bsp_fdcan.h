/**
 * @file bsp_fdcan.h
 * @author 自动生成
 * @brief FDCAN（Flexible Data-rate CAN）驱动接口头文件
 * @version 1.0
 * @date 2025-10-05
 *
 * 本文件为FDCAN驱动的接口定义，包含FDCAN实例结构体、初始化结构体、接收帧结构体及相关函数声明。
 * 统一注释风格，便于维护和文档生成。
 */

#ifndef BSP_FDCAN_H_
#define BSP_FDCAN_H_
#include "robot_config.h"
#ifdef USER_CAN_FD
#include "fdcan.h"
#include "bsp_typedef.h"
#define FDCAN_MAX_REGISTER_CNT 16

#pragma pack(1)
/**
 * @brief FDCAN实例结构体
 * @details 用于描述一个FDCAN外设实例及其相关配置和回调。
 */
typedef struct CanInstance_s
{
    char* topic_name;                                 ///< 实例名称
    FDCAN_HandleTypeDef *can_handle;                  ///< FDCAN句柄
    FDCAN_TxHeaderTypeDef tx_conf;                    ///< FDCAN报文发送配置
    uint16_t tx_id;                                   ///< 发送ID（FDCAN报文ID）
    uint8_t tx_buff[8];                               ///< 发送缓存，便于调试
    uint8_t* tx_buff_ptr;                             ///< 发送缓存指针
    uint16_t rx_id;                                   ///< 接收ID（FDCAN报文ID）
    uint8_t rx_buff[8];                               ///< 接收缓存，便于调试
    void (*can_module_callback)(struct CanInstance_s *); ///< 接收回调函数，用于解析接收数据
    void *parent_ptr;                                 ///< 使用CAN外设的父模块指针
} CanInstance_s;

/**
 * @brief FDCAN初始化配置结构体
 * @details 用于初始化FDCAN实例的参数配置。
 */
typedef struct
{
    char* topic_name;                                 ///< 实例名称
    uint8_t can_number;                               ///< CAN通道号（1,2,3分别对应FDCAN1,2,3）
    uint16_t tx_id;                                   ///< 发送ID
    uint16_t rx_id;                                   ///< 接收ID
    CAN_DataFrameTypeDef frame_type;                  ///< 数据帧类型
    void (*can_module_callback)(CanInstance_s *);     ///< 接收回调函数
    void *parent_ptr;                                 ///< 父模块指针
} CanInitConfig_s;
#pragma pack()

/**
 * @brief FDCAN接收帧结构体
 * @details 存储从FDCAN接收的消息，包括消息头和数据缓冲区。
 */
typedef struct {
    FDCAN_RxHeaderTypeDef Header;                     ///< FDCAN接收消息头
    uint8_t rx_buff[8];                               ///< 接收数据缓冲区
} FDCAN_RxFrame_TypeDef;

/**
 * @brief 注册并初始化FDCAN实例
 * @param config FDCAN初始化配置结构体指针
 * @return 成功返回FDCAN实例指针，失败返回NULL
 */
CanInstance_s* Can_Register(const CanInitConfig_s* config);

/**
 * @brief 通过外部发送缓冲区发送FDCAN数据
 * @param instance FDCAN实例指针
 * @param tx_buff 发送数据缓冲区指针
 * @return 发送成功返回true，失败返回false
 */
bool Can_Transmit_External_Tx_Buff(const CanInstance_s *instance, const uint8_t *tx_buff);
/**
 * @brief 通过CAN总线发送数据,为了避免大修MODULE而写的函数
 * 该函数将实例内部的发送缓冲区数据通过给定的CAN实例发送出去。如果发送成功，返回true；否则返回false。
 * @param instance 指向已注册的CanInstance_s结构体的指针，表示要使用的CAN实例
 * @return 如果数据发送成功则返回true，否则返回false
 */
bool Can_Transmit(const CanInstance_s *instance);
#endif
#endif
