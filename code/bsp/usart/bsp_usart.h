#ifndef BSP_USART_H
#define BSP_USART_H
#include "bsp_typedef.h"
#include <stdint.h>
#include "usart.h"

#define USART_MAX_REGISTER_CNT 10
typedef struct UsartInstance_s {
    char *topic_name;                                           // 实例名称
    TransferMode_e mode;                                        // 传输模式
    UART_HandleTypeDef* huart_handle;
    uint8_t* first_rx_buf;                                 // 第一个接收缓冲区
    uint8_t* second_rx_buf;                                // 第二个接收缓冲区
    CommunicationMode_e direction;                                  // 传输方向
    uint8_t tx_len;                                             // 发送数据长度
    uint8_t rx_len;                                             // 接收数据长度
    void (*usart_module_callback)(struct UsartInstance_s *,uint16_t Size);    // 回调函数
    void *parent_ptr;                                           // 使用USART外设的父模块指针
} UsartInstance_s;

typedef struct {
    char *topic_name;                                           // 实例名称
    UART_HandleTypeDef* huart_handle;
    TransferMode_e mode;                                        // 传输模式
    CommunicationMode_e direction;                                  // 传输方向
    uint8_t* first_rx_buf;                                 // 第一个接收缓冲区
    uint8_t* second_rx_buf;                                // 第二个接收缓冲区
    uint8_t tx_len;                                             // 发送数据长度
    uint8_t rx_len;                                             // 接收数据长度
    void (*usart_module_callback)(UsartInstance_s *,uint16_t Size);    // 回调函数
    void *parent_ptr;                                           // 使用USART外设的父模块指针
} UsartInitConfig_s;
UsartInstance_s* Usart_Register(const UsartInitConfig_s *config);
void Usart_RxDMA_DoubleBuffer_Init(UsartInstance_s * instance);

#endif
