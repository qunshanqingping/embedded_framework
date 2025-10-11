#ifndef BSP_USART_H
#define BSP_USART_H
#include "bsp_typedef.h"
#include <stdint.h>
#include "usart.h"
typedef struct UsartInstance_s {
    char *topic_name;                                           // 实例名称
    TransferMode_e mode;                                        // 传输模式
    UART_HandleTypeDef* huart_handle;
    uint8_t* first_rx_buf;                                 // 第一个接收缓冲区
    uint8_t* second_rx_buf;                                // 第二个接收缓冲区
    DirectionMode_e direction;                                  // 传输方向
    uint8_t tx_len;                                             // 发送数据长度
    uint8_t rx_len;                                             // 接收数据长度
    void (*usart_module_callback)(struct UsartInstance_s *);    // 回调函数
    void *parent_ptr;                                           // 使用USART外设的父模块指针
} UartInstance_s;

typedef struct {
    char *topic_name;                                           // 实例名称
    UART_HandleTypeDef* huart_handle;
    TransferMode_e mode;                                        // 传输模式
    DirectionMode_e direction;                                  // 传输方向
    uint8_t tx_len;                                             // 发送数据长度
    uint8_t rx_len;                                             // 接收数据长度
    void (*usart_module_callback)(struct UsartInstance_s *);    // 回调函数
    void *parent_ptr;                                           // 使用USART外设的父模块指针
} UartInitConfig_s;

void USART_RxDMA_DoubleBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength);

#endif
