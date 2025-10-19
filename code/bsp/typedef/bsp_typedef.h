/*
 * @file bsp_typedef.h
 * @brief BSP库基础类型定义头文件，提高复用率
 * @author Adonis Jin
 * @date 2025/08/07
 * @version 1.0.0
 */

#ifndef BSP_TYPEDEF_H
#define BSP_TYPEDEF_H

/**
 * @brief CAN数据帧类型枚举
 * @note 用于区分标准帧和扩展帧
 */
typedef enum {
    STD_FRAME = 0, // 标准帧
    EXT_FRAME = 1  // 扩展帧
} CAN_DataFrameTypeDef;

/**
 * @brief 通信外设传输模式枚举
 * @note 定义通信外设的数据传输方式
 */
typedef enum {
    BLOCK_MODE = 0,  //!< 阻塞模式，CPU等待传输完成
    IT_MODE = 1,     //!< 中断模式，CPU通过中断处理传输
    DMA_MODE = 2     //!< DMA模式，专用硬件完成数据传输
} TransferMode_e;

/**
 * @brief 通信模式枚举
 * @note 定义外设的通信方向
 */
typedef enum {
    RX_TX_MODE = 0,  //!< 全双工模式，同时收发
    RX_MODE = 1,     //!< 仅接收模式
    TX_MODE = 2      //!< 仅发送模式
} CommunicationMode_e;

/**
 * @brief 设备主从模式枚举
 * @note 定义通信协议的主/从工作模式
 */
typedef enum {
    SLAVE_MODE = 0,  //!< 从模式，设备响应主机命令
    MASTER_MODE = 1  //!< 主模式，设备控制通信
} DeviceMode_e;

/**
 * @brief 数据缓冲区模式枚举
 * @note 定义数据传输的缓冲策略
 */
typedef enum {
    SINGLE_BUFFER_MODE = 0,  //!< 单缓冲模式，使用一个缓冲区
    DOUBLE_BUFFER_MODE = 1   //!< 双缓冲模式，使用两个缓冲区实现连续传输
} BufferMode_e;

#endif // BSP_TYPEDEF_H
