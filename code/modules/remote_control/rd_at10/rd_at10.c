//
// Created by 29568 on 2025/11/23.
//

#include "rd_at10.h"

#include "fs_i6x.h"
#include "bsp_usart.h"
#include "sbus.h"
#include "memory_management.h"
#include "plf_log.h"
#include "watch_dog.h"
#include <string.h>
__attribute__((section(".axisram"), aligned(32))) uint8_t rd_sbus_rx_first_buff[SBUS_FRAME_SIZE];
__attribute__((section(".axisram"), aligned(32))) uint8_t rd_sbus_rx_second_buff[SBUS_FRAME_SIZE];

/**
 * @brief 解析两位开关
 * @param ch 通道值
 * @return 开关状态 0,1
 */
static inline uint8_t remoter_2stage_switch_parse(int16_t ch)
{
    if (ch < RD_RC_CH_VALUE_OFFSET)
    {
        return 0;
    }
    return 1;
}

/**
 * @brief 解析三位开关
 * @param ch 通道值
 * @return 开关状态 0,1,2
 */
static inline uint8_t remoter_3stage_switch_parse(int16_t ch)
{
    if (ch < RD_RC_CH_VALUE_OFFSET)
    {
        return 0;
    }
    if (ch == RD_RC_CH_VALUE_OFFSET)
    {
        return 1;
    }
    return 2;
}

static inline int16_t int16_deadline(int16_t Value){
    if (Value < RD_REMOTER_DEADLINE && Value > -RD_REMOTER_DEADLINE){
        Value = 0;
    }
    return Value;
}
/**
 * @brief 解析Sbus数据
 * @param buffer Sbus数据缓冲区指针
 * @param data I6x数据结构体指针
 */
static void Sbus_Decode(uint8_t* buffer, At10Data_s* data)
{
    Sbus_Frame_Parse(&data->rc, buffer);
    data->rx_freq.cnt_1s++;
    data->joy.right_x = int16_deadline((int16_t)(data->rc.ch[0] - RD_RC_CH_VALUE_OFFSET));
    data->joy.right_y = int16_deadline((int16_t)(data->rc.ch[1] - RD_RC_CH_VALUE_OFFSET));
    data->joy.left_y = int16_deadline((int16_t)(data->rc.ch[2] - RD_RC_CH_VALUE_OFFSET));
    data->joy.left_x = int16_deadline((int16_t)(data->rc.ch[3] - RD_RC_CH_VALUE_OFFSET));
    data->sw.a = remoter_2stage_switch_parse(data->rc.ch[4]);
    data->sw.b = remoter_2stage_switch_parse(data->rc.ch[5]);
    data->sw.c = remoter_3stage_switch_parse(data->rc.ch[6]);
    data->sw.d = remoter_2stage_switch_parse(data->rc.ch[7]);
    data->sw.e = remoter_2stage_switch_parse(data->rc.ch[8]);
    data->sw.f = remoter_2stage_switch_parse(data->rc.ch[9]);
    data->sw.g = remoter_3stage_switch_parse(data->rc.ch[10]);
    data->sw.h = remoter_2stage_switch_parse(data->rc.ch[11]);
}

/**
 * @brief Sbus接收回调函数
 * @param usart_instance USART实例指针
 * @param Size 接收数据长度
 */
// ReSharper disable once CppParameterMayBeConstPtrOrRef
static void Sbus_Callback(UsartInstance_s *usart_instance, uint16_t Size){
    if (usart_instance == NULL || usart_instance->parent_ptr == NULL || Size > SBUS_FRAME_SIZE)
    {
        return;
    }
    At10Instance_s *at10_instance = usart_instance->parent_ptr;
    if ((((DMA_Stream_TypeDef*)usart_instance->huart_handle->hdmarx->Instance)->CR & DMA_SxCR_CT) == RESET)
    {
        __HAL_DMA_DISABLE(usart_instance->huart_handle->hdmarx);
        ((DMA_Stream_TypeDef*)usart_instance->huart_handle->hdmarx->Instance)->CR |= DMA_SxCR_CT;
        __HAL_DMA_SET_COUNTER(usart_instance->huart_handle->hdmarx,SBUS_FRAME_SIZE * 2);
        if(Size == SBUS_FRAME_SIZE)
        {
            Sbus_Decode(usart_instance->first_rx_buf,&at10_instance->data);
        }
    }
    else
    {
        __HAL_DMA_DISABLE(usart_instance->huart_handle->hdmarx);
        ((DMA_Stream_TypeDef  *)usart_instance->huart_handle->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
        __HAL_DMA_SET_COUNTER(usart_instance->huart_handle->hdmarx,SBUS_FRAME_SIZE * 2);
        if(Size == SBUS_FRAME_SIZE)
        {
            Sbus_Decode(usart_instance->second_rx_buf,&at10_instance->data);
        }
    }
    __HAL_DMA_ENABLE(usart_instance->huart_handle->hdmarx);
}

static void Monitor_At10(WatchDogInstance_s * WatchDogInstance)
{
    At10Instance_s * instance = (At10Instance_s *)WatchDogInstance->parent_ptr;
    instance->data.rx_freq.frequency = instance->data.rx_freq.cnt_1s;
    instance->data.rx_freq.cnt_1s = 0;
}

At10Instance_s * At10_Register(UART_HandleTypeDef *huart){
    UsartInitConfig_s* usart_config = user_malloc_dt(sizeof(UsartInitConfig_s));
    WatchDogInitConfig_s* watch_dog_config = user_malloc_dt(sizeof(WatchDogInitConfig_s));
    At10Instance_s* at10_instance = user_malloc_dt(sizeof(At10Instance_s));
    if(usart_config == NULL || at10_instance == NULL || watch_dog_config == NULL){
        Log_Error("RD-AT10 SbusInstance UartConfig or at10_instance or watchdog Malloc Failed");
        user_free_dt(usart_config);
        user_free_dt(at10_instance);
        user_free_dt(watch_dog_config);
        return NULL;
    }
    memset(usart_config,0,sizeof(UsartInitConfig_s));
    memset(at10_instance,0,sizeof(At10Instance_s));
    memset(watch_dog_config,0,sizeof(WatchDogInitConfig_s));
    usart_config->topic_name = "RD-AT10";
    usart_config->huart_handle = huart;
    usart_config->mode = DMA_MODE;
    usart_config->direction = RX_MODE;
    usart_config->rx_len = SBUS_FRAME_SIZE;
    usart_config->first_rx_buf = rd_sbus_rx_first_buff;
    usart_config->second_rx_buf = rd_sbus_rx_second_buff;
    usart_config->usart_module_callback = NULL;
    usart_config->parent_ptr = at10_instance;
    usart_config->usart_module_callback = Sbus_Callback;
    at10_instance->usart_instance = Usart_Register(usart_config);
    user_free_dt(usart_config);
    Usart_RxDMA_DoubleBuffer_Init(at10_instance->usart_instance);

    watch_dog_config->topic_name = "RD-AT10";
    watch_dog_config->parent_ptr = at10_instance;
    watch_dog_config->watchdog_callback = Monitor_At10;
    at10_instance->watchdog_instance = WatchDog_Register(watch_dog_config);
    user_free_dt(watch_dog_config);

    return at10_instance;
}