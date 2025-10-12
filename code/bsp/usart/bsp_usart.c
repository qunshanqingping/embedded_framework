#include "bsp_usart.h"

#include <string.h>

#include "plf_log.h"
#include "memory_management.h"

uint8_t usart_idx = 0;
UsartInstance_s* usart_instance[USART_MAX_REGISTER_CNT];

/**
  * @brief  USART Rx DMA 双缓冲区初始化
  * @param  huart: 指向 UART_HandleTypeDef 结构体的指针，表示要配置的 USART 实例。
  * @param  DstAddress: 指向第一个接收缓冲区的指针，用于存储接收到的数据。
  * @param  SecondMemAddress: 指向第二个接收缓冲区的指针，用于存储接收到的数据。
  * @param  DataLength: 每个缓冲区的大小（以字节为单位）。
  * @retval None
  * @note   在调用此函数之前，确保 huart 和 DMA 已正确初始化。
  *         此函数配置 USART 接收模式为 IDLE，并启用 DMA 双缓冲区模式。
  *         当接收到数据时，DMA 会自动在两个缓冲区之间切换。
  */
static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *huart, uint8_t *DstAddress,
                                         uint8_t *SecondMemAddress, uint8_t DataLength) {

    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

    huart->RxEventType = HAL_UART_RXEVENT_IDLE;

    huart->RxXferSize    = (uint32_t)DataLength * 2;

    SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);

    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    __HAL_DMA_DISABLE(huart->hdmarx);

    HAL_DMAEx_MultiBufferStart(huart->hdmarx, (uint32_t)&huart->Instance->RDR, (uint32_t)DstAddress, (uint32_t)SecondMemAddress, (uint32_t)DataLength * 2);
}
void Usart_RxDMA_DoubleBuffer_Init( UsartInstance_s * instance){
    USART_RxDMA_MultiBuffer_Init(instance->huart_handle,
                                 instance->first_rx_buf,
                                 instance->second_rx_buf,
                                 instance->rx_len);
}
UsartInstance_s* Usart_Register(const UsartInitConfig_s *config){

    UsartInstance_s *instance = user_malloc(sizeof(UsartInstance_s));
    if(instance == NULL){
        Log_Error("%s UartInstance Malloc Failed", config->topic_name);
        return NULL;
    }
    memset(instance, 0, sizeof(UsartInstance_s));
    instance->topic_name = config->topic_name;
    instance->huart_handle = config->huart_handle;
    instance->rx_len = config->rx_len;
    instance->tx_len = config->tx_len;
    instance->first_rx_buf = config->first_rx_buf;
    instance->second_rx_buf = config->second_rx_buf;

    instance->usart_module_callback = config->usart_module_callback;
    instance->parent_ptr = config->parent_ptr;
    usart_instance[usart_idx++] = instance;
    return instance;
}

// ReSharper disable once CppParameterMayBeConstPtrOrRef
// ReSharper disable once CppParameterNeverUsed
// ReSharper disable once CppParameterMayBeConst
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint8_t i = 0; i < usart_idx; ++i)
    { // find the instance which is being handled
        if (huart == usart_instance[i]->huart_handle)
        { // call the callback function if it is not NULL
            if (usart_instance[i]->usart_module_callback != NULL)
            {
                usart_instance[i]->usart_module_callback(usart_instance[i], Size);
            }
            return; // break the loop
        }
    }
}
