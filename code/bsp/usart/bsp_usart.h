/**
 *   @file bsp_usart.h
 *   @brief 串口通讯的实现
 *   @author Wenxin HU
 *   @date 25-7-11
 *   @version 0.1
 *   @note
 */
#ifndef BSP_USART_H
#define BSP_USART_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp_typedef.h"
#include "usart.h"
#define USART_MAX_CNT 10 //
/* 目前C板可以直接连接的UART */
// #define UART_1 //如果使用UART1,需要定义此宏
// #define UART_3 //如果使用UART3,需要定义此宏
// #define UART_6 //如果使用UART6,需要定义此宏
/* 为了兼容喵板等开发版设置的其他UART */
// #define UART_4
// #define UART_5
// #define UART_7
// #define UART_8
// #define UART_9
// #define USART_1
// #define USART_2
// #define USART_3
// #define USART_6
// #define USART_10


/* 串口的通讯模式 */

/* UART实例化结构体 */
typedef struct _UartInstance_s
{
    UART_HandleTypeDef* uart_handle; // 串口句柄
    TransferMode_e transfer_mode; // 串口通讯模式
    DirectionMode_e direction_mode;
    BufferMode_e buffer_mode;

    uint8_t rx_len; // 接收长度
    uint8_t tx_len; // 发送长度 小于50Byte
    uint8_t* tx_first_buff; // 发送缓存
    uint8_t* tx_second_buff;
    uint8_t* rx_first_buff; // 接收缓存
    uint8_t* rx_second_buff;

    void (*uart_module_callback)( void* parent,uint16_t size); // 接收的回调函数,用于解析接收到的数据
    void* parent; // 使用uart外设的模块指针(即id指向的模块拥有此uart实例,是父子关系)
} UartInstance_s;

/* UART实例初始化结构体,将此结构体指针传入注册函数 */
typedef struct
{
    UART_HandleTypeDef* uart_handle; // 串口句柄
    TransferMode_e transfer_mode; // 串口通讯模式
    DirectionMode_e direction_mode;
    BufferMode_e buffer_mode;
    uint8_t tx_len;
    uint8_t rx_len; // 接收长度 小于50Byte
    void (*uart_module_callback)( void* parent,uint16_t size); // 接收的回调函数,用于解析接收到的数据
    void* parent_pointer; // 使用uart外设的模块指针(即id指向的模块拥有此uart实例,是父子关系)
} UartConfig_s;

/**
 * @file bsp_usart.h
 * @brief UART实例注册函数
 * @param config UART初始化配置结构体指针
 * @return instance指针--注册成功   NULL--注册失败
 * @date 2025-06-30
 */
UartInstance_s* Uart_Register(UartConfig_s* config);

/**
 * @file bsp_usart.h
 * @brief UART发送数据函数
 * @param uart_instance UART实例指针
 * @param data 发送数据指针
 * @return true--发送成功   false--发送失败
 * @date 2025-06-30
 */
bool Uart_Transmit(UartInstance_s* uart_instance, uint8_t* data);

bool Uart_Blocking_Receive(UartInstance_s* uart_instance);
#endif // BSP_USART_H
