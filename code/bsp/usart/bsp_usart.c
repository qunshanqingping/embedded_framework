#include "bsp_usart.h"

#include "plf_log.h"

// static int uart_receive_dma_no_it(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size)
// {
//     uint32_t tmp = 0;
//     tmp = huart->RxState;
//
//     /* ÅÐ¶Ï´®¿ÚÊÇ·ñÒÑ¾­³õÊ¼»¯Íê³É */
//     if (tmp == HAL_UART_STATE_READY)
//     {
//         /* ¼ì²âÓÃ»§ÊäÈëµÄÊý¾ÝÊÇ·ñÕýÈ· */
//         if ((pData == NULL) || (Size == 0))
//             return HAL_ERROR;
//
//         huart->pRxBuffPtr = pData;
//         huart->RxXferSize = Size;
//         huart->ErrorCode = HAL_UART_ERROR_NONE;
//
//         /* Ê¹ÄÜDMAÍ¨µÀ */
//         HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->RDR, (uint32_t)pData, Size);
//
//         /* ¿ªÆôDMA´«Êä ½«UART CR3 ¼Ä´æÆ÷ÖÐµÄ DMARÎ» ÖÃ¸ß */
//         SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
//
//         return HAL_OK;
//     }
//     else
//         return HAL_BUSY;
// }、

/**
 *
 * @param usartInstance 串口实例指针
 * @return true 开启 对应串口DMA | false 开启失败
 */
static bool Uart_Receive_DMA_NO_IT(UartInstance_s *usartInstance)
{
    uint32_t tmp = 0;
    tmp = usartInstance->huart_handle->RxState;
    if (tmp == HAL_UART_STATE_READY)
    {
        if (usartInstance->ring_buff.data == NULL || usartInstance->ring_buff.element_size == 0) {
            Log_Error("%s ring_buff or rx_len is NULL");
            return false;
        }
        usartInstance->huart_handle->pRxBuffPtr = usartInstance->ring_buff.data;
        usartInstance->huart_handle->RxXferSize = usartInstance->ring_buff.element_size;
        usartInstance->huart_handle->ErrorCode = HAL_UART_ERROR_NONE;

        HAL_DMA_Start(usartInstance->huart_handle->hdmarx,(uint32_t)&usartInstance->huart_handle->Instance->RDR,
            (uint32_t)usartInstance->ring_buff.data,usartInstance->ring_buff.element_size);

        SET_BIT(usartInstance->huart_handle->Instance->CR3,USART_CR3_DMAR);
        return true;
    }
        return false;
}
// /**
//   * @brief	¿ÕÏÐÖÐ¶Ï³õÊ¼»¯º¯Êý
//   * @param	huart:UART¾ä±úÖ¸Õë
//   * @retval	none
//   */
// void uart_receive_init(UART_HandleTypeDef *huart)
// {
//     if (huart == &huart5)
//     {
//         /* Çå³ý¿ÕÏÐÖÐ¶Ï±êÖ¾Î» */
//         __HAL_UART_CLEAR_IDLEFLAG(&huart5);
//         /* ¿ªÆô´®¿Ú¿ÕÏÐÖÐ¶Ï */
//         __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
//         /* ¿ªÆôDMA½ÓÊÕ Ö¸¶¨½ÓÊÕ³¤¶ÈºÍÊý¾ÝµØÖ· */
//         uart_receive_dma_no_it(&huart5, usart5_buf, USART5_MAX_LEN);
//     }
// }

static void Uart_Receive_Init(UartInstance_s *usartInstance) {
    __HAL_UART_CLEAR_IDLEFLAG(usartInstance->huart_handle);
    __HAL_UART_ENABLE_IT(usartInstance->huart_handle, UART_IT_IDLE);

}