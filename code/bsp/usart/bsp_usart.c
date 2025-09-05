#include "bsp_usart.h"
#include "basic_math.h"
uint8_t id = 0; // 用于标识UART实例的唯一id,从0开始
UartInstance_s *uart_instance[USART_MAX_CNT]; // UART实例数组,用于存储UART注册的实例
/**
 *@brief DMA双缓冲接收初始化函数
 * @param huart uart句柄
 * @param DstAddress 第一个缓冲区地址
 * @param SecondMemAddress 第二个缓冲区地址
 * @param DataLength 接受数据的总长度
 * @author 辽宁科技大学 王草凡
 */
static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *huart,  const uint32_t *DstAddress, const uint32_t *SecondMemAddress, uint32_t DataLength)
{

    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

    huart->RxXferSize    = DataLength;

    SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);

    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    do{
        __HAL_DMA_DISABLE(huart->hdmarx);
    }while(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);

    /* Configure the source memory Buffer address  */
    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->RDR;

    /* Configure the destination memory Buffer address */
    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;

    /* Configure DMA Stream destination address */
    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;

    /* Configure the length of data to be transferred from source to destination */
    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = DataLength;

    /* Enable double memory buffer */
    SET_BIT(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);

    /* Enable DMA */
    __HAL_DMA_ENABLE(huart->hdmarx);
}


/**
 *@brief DMA单缓冲发送初始化函数,形式上封装
 *@param huart uart句柄
 */
void USART_RxDMA_SingleBuffer_Init(UART_HandleTypeDef *huart)
{
    SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);
}

UartInstance_s *Uart_Register(UartConfig_s *config)
{
    if (config == NULL || id >= USART_MAX_CNT)
    {
        return NULL; // 如果空间已满或者没有配置信息，返回NULL
    }
    for (uint8_t i = 0; i < id; i++)
    {
        if (uart_instance[i]->uart_handle == config->uart_handle)
        {
            return NULL; // 如果已经注册过了，返回NULL
        }
    }

    // 开始分配空间
    UartInstance_s *instance = (UartInstance_s *)user_malloc(sizeof(UartInstance_s));
    memset(instance, 0, sizeof(UartInstance_s)); // 清空空间
    if (instance == NULL)
    {
        user_free(instance); // 分配失败，释放内存
        return NULL;
    }

    instance->uart_handle = config->uart_handle; // 设置串口句柄
    instance->transfer_mode = config->transfer_mode;
    instance->direction_mode = config->direction_mode;
    instance->buffer_mode = config->buffer_mode;
    instance->parent = config->parent_pointer;
    // 根据传输方向模式分配缓冲区
    if (instance->direction_mode == RX_TX_MODE)
    {
        {
            instance->tx_len = config->tx_len;
            instance->rx_len = config->rx_len;
            instance->rx_first_buff = (uint8_t *)user_malloc(instance->rx_len);
            // memset(instance->rx_first_buff, 0, instance->rx_len);
            instance->tx_first_buff = (uint8_t *)user_malloc(instance->tx_len);
            // memset(instance->tx_first_buff, 0, instance->tx_len);
            if (instance->rx_first_buff == NULL || instance->tx_first_buff == NULL)
            {
                return NULL;
            }
            if (instance->buffer_mode == DOUBLE_BUFFER_MODE)
            {
                instance->rx_second_buff = (uint8_t *)user_malloc(instance->rx_len);
                memset(instance->rx_second_buff, 0, instance->rx_len);
                instance->tx_second_buff = (uint8_t *)user_malloc(instance->tx_len);
                memset(instance->tx_second_buff, 0, instance->tx_len);
                if (instance->rx_second_buff == NULL || instance->tx_second_buff == NULL)
                {
                    return NULL;
                }
            }
        }
    }
    else if (instance->direction_mode == RX_MODE)
    {
        instance->rx_len = config->rx_len;
        instance->rx_first_buff = (uint8_t *)user_malloc(instance->rx_len);
        memset(instance->rx_first_buff, 0, instance->rx_len);
        if (instance->rx_first_buff == NULL)
        {
            return NULL;
        }
        // 如果启用双缓冲模式
        if (instance->buffer_mode == DOUBLE_BUFFER_MODE)
        {
            instance->rx_second_buff = (uint8_t *)user_malloc(instance->rx_len);
            memset(instance->rx_second_buff, 0, instance->rx_len);
            if (instance->rx_second_buff == NULL)
            {
                return NULL;
            }
        }
    }

    else
    {
        instance->tx_first_buff = (uint8_t *)user_malloc(instance->tx_len);
        memset(instance->tx_first_buff, 0, instance->tx_len);
        if (instance->tx_first_buff == NULL)
        {
            return NULL;
        }
        if (instance->buffer_mode == DOUBLE_BUFFER_MODE)
        {
            instance->tx_second_buff = (uint8_t *)user_malloc(instance->tx_len);
            memset(instance->tx_second_buff, 0, instance->tx_len);
            if (instance->tx_second_buff == NULL)
            {
                return NULL;
            }
        }
    }


    if (instance->direction_mode != TX_MODE)
    {
        instance->tx_len = config->tx_len;
        if (instance->buffer_mode == DOUBLE_BUFFER_MODE)
        {
            USART_RxDMA_MultiBuffer_Init(instance->uart_handle, (uint32_t*)instance->rx_first_buff,(uint32_t*) instance->rx_second_buff, 2*instance->rx_len);
        }
        else
        {
            USART_RxDMA_SingleBuffer_Init(instance->uart_handle);
        }
    }
    instance->parent = config->parent_pointer;
    instance->uart_module_callback = config->uart_module_callback; // 设置回调函数
    uart_instance[id++] = instance; // 将实例添加到UART实例数组中
    return instance;
}

static void MODULE_UARTx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
    for (uint8_t i = 0; i < id; i++)
    {
        if (uart_instance[i]->uart_handle == huart)
        {
            uart_instance[i]->uart_module_callback(uart_instance[i]->parent,Size);
        }
    }

}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t size)
{
    MODULE_UARTx_RxEventCallback(huart,size);
    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

    /* Enable IDLE interrupt */
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    /* Enable the DMA transfer for the receiver request */
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    /* Enable DMA */
    __HAL_DMA_ENABLE(huart->hdmarx);
}